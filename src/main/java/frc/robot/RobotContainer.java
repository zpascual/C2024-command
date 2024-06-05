// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private double maxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    private double maxAngularRate = 4 * Math.PI; // 3/4 of a rotation per second

    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    public final Drive drivetrain = TunerConstants.DriveTrain;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * OperatorConstants.DEADBAND)
            .withRotationalDeadband(maxAngularRate * OperatorConstants.DEADBAND)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake driveLockRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric driveForwardRequest = new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.PointWheelsAt drivePointRequest = new SwerveRequest.PointWheelsAt();

    private final RobotState robotState = new RobotState(maxSpeed);

    public RobotContainer() {
        configureBindings();
    }
    

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> driveRequest
                        .withVelocityX(-driverController.getLeftY() * maxSpeed)
                        .withVelocityY(driverController.getLeftX() * maxSpeed)
                        .withRotationalRate(-driverController.getRightY() * maxAngularRate)
                ).ignoringDisable(true)
        );

        driverController.start().and(driverController.back()).onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(robotState::updateOdometry);

        /**
         * Binds for drivetrain characterization
         */
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
