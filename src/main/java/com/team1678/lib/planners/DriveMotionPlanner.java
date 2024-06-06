package com.team1678.lib.planners;

import com.team1678.frc2024.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMotionPlanner {
    private static final double MAX_DX = 0.0127; // m
    private static final double MAX_DY = 0.0127; // m
    private static final double MAX_D_THETA = Math.toRadians(1.0);

    // Pure Pursuit Constants
    public static final double PATH_LOOKAHEAD_TIME = 0.1; // From 1323 (2019)
    public static final double PATH_MIN_LOOKAHEAD_DISTANCE = 0.3; // From 1323 (2019)
    public static final double ADAPTIVE_PATH_MIN_LOOKAHEAD_DISTANCE = 0.15;
    public static final double ADAPTIVE_PATH_MAX_LOOKAHEAD_DISTANCE = 0.61;
    public static final double ADAPTIVE_ERROR_LOOKAHEAD_COEFFICIENT = 0.01;

    public enum FollowerType {
        PID,
        PURE_PURSUIT,
    }

    FollowerType mFollowerType = FollowerType.PURE_PURSUIT;

    public void setFollowerType(FollowerType type) {
        mFollowerType = type;
    }

    private double defaultCook = 0.5;
    private boolean useDefaultCook = true;

    public void setDefaultCook(double new_value) {
        defaultCook = new_value;
    }

    TrajectoryIterator<TimedState<Pose2dWithMotion>> mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedState<Pose2dWithMotion> mLastSetpoint = null;
    public TimedState<Pose2dWithMotion> mSetpoint = new TimedState<>(Pose2dWithMotion.identity());
    Pose2d mError = Pose2d.identity();

    ErrorTracker mErrorTracker = new ErrorTracker(15 * 100);
    Translation2d mTranslationalError = Translation2d.identity();
    Rotation2d mPrevHeadingError = Rotation2d.identity();
    Pose2d mCurrentState = Pose2d.identity();

    double mCurrentTrajectoryLength = 0.0;
    double mTotalTime = Double.POSITIVE_INFINITY;
    double mStartTime = Double.POSITIVE_INFINITY;
    ChassisSpeeds mOutput = new ChassisSpeeds();

    Lookahead mSpeedLookahead = null;

    // PID controllers for path following
    SynchronousPIDF mXPIDF;
    SynchronousPIDF mYPIDF;
    SynchronousPIDF mHeadingPIDF;

    double mDt = 0.0;

    public DriveMotionPlanner() {}

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithMotion>> trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        mLastSetpoint = null;
        useDefaultCook = true;
        mSpeedLookahead = new Lookahead(
                ADAPTIVE_PATH_MIN_LOOKAHEAD_DISTANCE,
                ADAPTIVE_PATH_MAX_LOOKAHEAD_DISTANCE,
                0.0,
                Constants.SwerveConstants.maxAutoSpeed);
        mCurrentTrajectoryLength =
                mCurrentTrajectory.trajectory().getLastPoint().state().t();
        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getPoint(i).state().velocity() > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getPoint(i).state().velocity() < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
    }

    public void reset() {
        mErrorTracker.reset();
        mTranslationalError = Translation2d.identity();
        mPrevHeadingError = Rotation2d.identity();
        mLastSetpoint = null;
        mOutput = new ChassisSpeeds();
        mLastTime = Double.POSITIVE_INFINITY;
    }


    protected ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds) {
        // Feedback on longitudinal error (distance).
        final double PATH_K = 2.4; // 2.4;/* * Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)*/;//0.15;
        final double PATH_K_THETA = 3.0;

        Twist2d pid_error = Pose2d.log(mError);

        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond + PATH_K * pid_error.dx;
        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond + PATH_K * pid_error.dy;
        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond + PATH_K_THETA * pid_error.dtheta;
        return chassisSpeeds;
    }

    protected ChassisSpeeds updatePurePursuit(Pose2d current_state, double feedforwardOmegaRadiansPerSecond) {
        double lookahead_time = PATH_LOOKAHEAD_TIME;
        final double kLookaheadSearchDt = 0.01;
        TimedState<Pose2dWithMotion> lookahead_state =
                mCurrentTrajectory.preview(lookahead_time).state();
        double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mSetpoint.velocity()) + ADAPTIVE_ERROR_LOOKAHEAD_COEFFICIENT * mError.getTranslation().norm();
        SmartDashboard.putNumber("PurePursuit/Error", mError.getTranslation().norm());
        // Find the Point on the Trajectory that is Lookahead Distance Away
        while (actual_lookahead_distance < adaptive_lookahead_distance
                && mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        }

        // If the Lookahead Point's Distance is less than the Lookahead Distance transform it so it is the lookahead
        // distance away
        if (actual_lookahead_distance < adaptive_lookahead_distance) {
            lookahead_state = new TimedState<>(
                    new Pose2dWithMotion(
                            lookahead_state
                                    .state()
                                    .getPose()
                                    .transformBy(Pose2d.fromTranslation(new Translation2d(
                                            (mIsReversed ? -1.0 : 1.0)
                                                    * (PATH_MIN_LOOKAHEAD_DISTANCE - actual_lookahead_distance),
                                            0.0))),
                            0.0),
                    lookahead_state.t(),
                    lookahead_state.velocity(),
                    lookahead_state.acceleration());
        }
        SmartDashboard.putNumber("PurePursuit/ActualLookaheadDist", actual_lookahead_distance);
        SmartDashboard.putNumber("PurePursuit/AdaptiveLookahead", adaptive_lookahead_distance);
        LogUtil.recordPose2d("PurePursuit/LookaheadState", lookahead_state.state().getPose());
        SmartDashboard.putNumber("PurePursuit/RemainingProgress", mCurrentTrajectory.getRemainingProgress());
        SmartDashboard.putNumber("PurePursuit/PathVelocity", lookahead_state.velocity());

        if (lookahead_state.velocity() == 0.0) {
            mCurrentTrajectory.advance(Double.POSITIVE_INFINITY);
            return new ChassisSpeeds();
        }

        // Find the vector between robot's current position and the lookahead state
        Translation2d lookaheadTranslation = new Translation2d(
                current_state.getTranslation(), lookahead_state.state().getTranslation());

        // Set the steering direction as the direction of the vector
        Rotation2d steeringDirection = lookaheadTranslation.direction();

        // Convert from field-relative steering direction to robot-relative
        steeringDirection = steeringDirection.rotateBy(current_state.inverse().getRotation());

        // Use the Velocity Feedforward of the Closest Point on the Trajectory
        double normalizedSpeed = Math.abs(mSetpoint.velocity()) / Constants.SwerveConstants.maxAutoSpeed;

        // The Default Cook is the minimum speed to use. So if a feedforward speed is less than defaultCook, the robot
        // will drive at the defaultCook speed
        if (normalizedSpeed > defaultCook || mSetpoint.t() > (mCurrentTrajectoryLength / 2.0)) {
            useDefaultCook = false;
        }
        if (useDefaultCook) {
            normalizedSpeed = defaultCook;
        }

        SmartDashboard.putNumber("PurePursuit/NormalizedSpeed", normalizedSpeed);


        // Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate (Vx, Vy) in Robot Frame
        final Translation2d steeringVector =
                new Translation2d(steeringDirection.cos() * normalizedSpeed, steeringDirection.sin() * normalizedSpeed);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                steeringVector.x() * Constants.SwerveConstants.maxAutoSpeed,
                steeringVector.y() * Constants.SwerveConstants.maxAutoSpeed,
                feedforwardOmegaRadiansPerSecond);

        // Use the PD-Controller for To Follow the Time-Parametrized Heading
        final double kThetakP = 3.5;
        final double kThetakD = 0.0;
        final double kPositionkP = 2.0;

        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond
                + kPositionkP * mError.getTranslation().x();
        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond
                + kPositionkP * mError.getTranslation().y();
        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond
                + (kThetakP * mError.getRotation().getRadians())
                + kThetakD * ((mError.getRotation().getRadians() - mPrevHeadingError.getRadians()) / mDt);
        return chassisSpeeds;
    }

    public ChassisSpeeds update(double timestamp, Pose2d current_state, Translation2d current_velocity) {
        if (mCurrentTrajectory == null) return null;

        if (!Double.isFinite(mLastTime)) mLastTime = timestamp;
        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint<TimedState<Pose2dWithMotion>> sample_point;
        mCurrentState = current_state;

        if (!isDone()) {
            // Compute error in robot frame
            mPrevHeadingError = mError.getRotation();
            mError = current_state.inverse().transformBy(mSetpoint.state().getPose());
            mErrorTracker.addObservation(mError);

            if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
                sample_point = mCurrentTrajectory.advance(mDt);
                // RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
                mSetpoint = sample_point.state();

                final double velocity_m = mSetpoint.velocity();
                // Field relative
                var course = mSetpoint.state().getCourse();
                Rotation2d motion_direction = course.isPresent() ? course.get() : Rotation2d.identity();
                // Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.
                motion_direction = current_state.getRotation().inverse().rotateBy(motion_direction);

                mOutput = new ChassisSpeeds(
                        motion_direction.cos() * velocity_m,
                        motion_direction.sin() * velocity_m,
                        // Need unit conversion because Pose2dWithMotion heading rate is per unit distance.
                        velocity_m * mSetpoint.state().getHeadingRate());
            } else if (mFollowerType == FollowerType.RAMSETE) {
                sample_point = mCurrentTrajectory.advance(mDt);
                // RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
                mSetpoint = sample_point.state();
                // mOutput = updateRamsete(sample_point.state(), current_state, current_velocity);
            } else if (mFollowerType == FollowerType.PID) {
                sample_point = mCurrentTrajectory.advance(mDt);
                // RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
                mSetpoint = sample_point.state();

                final double velocity_m = mSetpoint.velocity();
                // Field relative
                var course = mSetpoint.state().getCourse();
                Rotation2d motion_direction = course.isPresent() ? course.get() : Rotation2d.identity();
                // Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.
                motion_direction = current_state.getRotation().inverse().rotateBy(motion_direction);

                var chassis_speeds = new ChassisSpeeds(
                        motion_direction.cos() * velocity_m,
                        motion_direction.sin() * velocity_m,
                        // Need unit conversion because Pose2dWithMotion heading rate is per unit distance.
                        velocity_m * mSetpoint.state().getHeadingRate());
                // PID is in robot frame
                mOutput = updatePIDChassis(chassis_speeds);
            } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
                double searchStepSize = 1.0;
                double previewQuantity = 0.0;
                double searchDirection = 1.0;
                double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
                double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
                searchDirection = Math.signum(reverseDistance - forwardDistance);
                while (searchStepSize > 0.001) {
                    SmartDashboard.putNumber("PurePursuit/PreviewDist", distance(current_state, previewQuantity));
                    if (Util.epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.0003937)) break;
                    while (
                        /* next point is closer than current point */ distance(
                            current_state, previewQuantity + searchStepSize * searchDirection)
                            < distance(current_state, previewQuantity)) {
                        /* move to next point */
                        previewQuantity += searchStepSize * searchDirection;
                    }
                    searchStepSize /= 10.0;
                    searchDirection *= -1;
                }
                SmartDashboard.putNumber("PurePursuit/PreviewQtd", previewQuantity);
                sample_point = mCurrentTrajectory.advance(previewQuantity);
                // RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
                mSetpoint = sample_point.state();
                mOutput = updatePurePursuit(current_state, 0.0);
            }
        } else {
            if (mCurrentTrajectory.trajectory().getLastPoint().state().velocity() == 0.0) {
                mOutput = new ChassisSpeeds();
            }
        }

        return mOutput;
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    public synchronized Translation2d getTranslationalError() {
        return new Translation2d(
                mError.getTranslation().x(), mError.getTranslation().y());
    }

    public synchronized Rotation2d getHeadingError() {
        return mError.getRotation();
    }

    private double distance(Pose2d current_state, double additional_progress) {
        return mCurrentTrajectory
                .preview(additional_progress)
                .state()
                .state()
                .getPose()
                .distance(current_state);
    }

    public synchronized TimedState<Pose2dWithMotion> getSetpoint() {
        return mSetpoint;
    }

    public synchronized ErrorTracker getErrorTracker() {
        return mErrorTracker;
    }
}
