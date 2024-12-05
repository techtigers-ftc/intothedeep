package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * A command group that moves the intake system to the transfer position
 */
public class IntakeTransferCommandGroup extends SequentialCommandGroup {
    /**
     * Creates a new IntakeTransferCommandGroup
     * @param intake the intake subsystem
     */
    public IntakeTransferCommandGroup(IntakeSubsystem intake) {
        addCommands(
                intake.getClawCommand(false),
                intake.getWristCommand(180, 90, 500),
                new ParallelCommandGroup(
                        intake.getWristCommand(0, 90, 0),
                        intake.getSlidesCommand(1, 0.5)
                )
        );
    }

}
