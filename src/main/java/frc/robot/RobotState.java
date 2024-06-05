package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class RobotState {
    private final double maxSpeed;

    public RobotState(double maxSpeed) {
        this.maxSpeed = maxSpeed;
        SignalLogger.start();
    }

    private Pose2d m_lastPose = new Pose2d();
    private double lastTimestamp = Utils.getCurrentTimeSeconds();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
            m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
            m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    public void updateOdometry(SwerveDrivetrain.SwerveDriveState state) {

        Pose2d robotPose = state.Pose;

        /* Telemeterize the pose */
        Logger.recordOutput("Drive/Odometry", robotPose);

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTimestamp;
        lastTimestamp = currentTime;
        Translation2d distanceDiff = robotPose.minus(m_lastPose).getTranslation();
        m_lastPose = robotPose;

        Translation2d velocities = distanceDiff.div(diffTime);

        /* Telemeterize the module's states */
        for (int i = 0; i < 4; ++i) {
           var moduleState = state.ModuleStates[i];
           var angle = moduleState.angle;
           var speedRatio = moduleState.speedMetersPerSecond / (2 * maxSpeed);

           m_moduleSpeeds[i].setAngle(angle);
           m_moduleDirections[i].setAngle(angle);
           m_moduleSpeeds[i].setLength(speedRatio);

           Logger.recordOutput("Module " + i, m_moduleMechanisms[i]);
        }

        SignalLogger.writeDoubleArray("odometry", new double[] {robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees()});
        SignalLogger.writeDouble("odom period", state.OdometryPeriod, "seconds");
    }

}
