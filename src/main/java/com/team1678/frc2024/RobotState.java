package com.team1678.frc2024;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class RobotState {
    private final double maxSpeed;

    public RobotState(double maxSpeed) {
        this.maxSpeed = maxSpeed;
        SignalLogger.start();
    }

    private Pose2d lastPose = new Pose2d();
    private double lastTimestamp = Utils.getCurrentTimeSeconds();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
            moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
            moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    public void updateOdometry(SwerveDrivetrain.SwerveDriveState state) {

        Pose2d robotPose = state.Pose;
        Logger.recordOutput("Drive/Odometry", robotPose);

        /* Telemeter the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTimestamp;
        lastTimestamp = currentTime;
        Translation2d distanceDiff = robotPose.minus(lastPose).getTranslation();
        lastPose = robotPose;

        Translation2d velocities = distanceDiff.div(diffTime);

        Logger.recordOutput("Drive/Velocity", velocities.getNorm());
        Logger.recordOutput("Drive/Velocity X", velocities.getX());
        Logger.recordOutput("Drive/Velocity Y", velocities.getY());
        Logger.recordOutput("Drive/Odom Update Freq", state.OdometryPeriod);

        /* Telemeter the module's states */
        for (int i = 0; i < 4; ++i) {
           var moduleState = state.ModuleStates[i];
           var angle = moduleState.angle;
           var speedRatio = moduleState.speedMetersPerSecond / (2 * maxSpeed);

           moduleSpeeds[i].setAngle(angle);
           moduleDirections[i].setAngle(angle);
           moduleSpeeds[i].setLength(speedRatio);

           Logger.recordOutput("Drive/Module " + i, moduleMechanisms[i]);
        }

        SignalLogger.writeDoubleArray("Odometry", new double[] {robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees()});
        SignalLogger.writeDouble("Odometry period", state.OdometryPeriod, "seconds");
    }

    public Supplier<Pose2d> getOdomPose() {
        return () -> lastPose;
    }

}
