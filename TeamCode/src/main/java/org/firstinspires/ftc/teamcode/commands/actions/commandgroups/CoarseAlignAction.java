package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.drive.CoarseAlignDriveAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeSlidesCoarseAlignAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

public class CoarseAlignAction extends ParallelCommandGroup {
    public CoarseAlignAction(IntakeSubsystem intake, DriveSubsystem drive, RobotState robotState) {
        addRequirements(intake, drive);
        addCommands(
                new CoarseAlignDriveAction(drive, robotState, 0.75),
                new IntakeSlidesCoarseAlignAction(intake, robotState, 0.5)
        );
    }
}
