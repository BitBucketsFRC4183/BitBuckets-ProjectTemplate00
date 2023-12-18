package org.bitbuckets.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.bitbuckets.DriveSubsystem;
import org.bitbuckets.Robot;

public class RotateForeverCommand extends CommandBase {

    final DriveSubsystem robot;

    public RotateForeverCommand(DriveSubsystem robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        super.initialize();

    }

    @Override
    public void execute(){
        robot.driveAt(0, 1);
    }

    @Override
    public void end(boolean interrupted) {
        robot.driveAt(0, 0);
    }
}
