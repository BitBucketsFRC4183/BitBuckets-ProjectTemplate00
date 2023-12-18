package org.bitbuckets.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.bitbuckets.DriveSubsystem;
import org.bitbuckets.Robot;

public class CircleCommand extends SequentialCommandGroup {
    public CircleCommand(DriveSubsystem robot) {
        super(
                Commands.waitSeconds(2),
                Commands.deadline(Commands.waitSeconds(2), new RotateForeverCommand(robot)),
                Commands.waitSeconds(2),
                Commands.deadline(Commands.waitSeconds(2), new RotateForeverCommand(robot))
        );
    }
}
