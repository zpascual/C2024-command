package com.team1678.frc2024.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import com.team1678.frc2024.subsystems.Drive;

public class FollowPathCommand extends Command {

    private final Drive drive;
    private final PathPlannerPath path;

    public FollowPathCommand(Drive drive, String pathName) {
        this.drive = drive;
        addRequirements(drive);
        this.path = PathPlannerPath.fromChoreoTrajectory(pathName);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.seedFieldRelative(path.getStartingDifferentialPose());
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.lock();
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
