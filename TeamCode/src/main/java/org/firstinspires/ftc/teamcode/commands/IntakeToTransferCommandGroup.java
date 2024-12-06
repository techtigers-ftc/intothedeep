package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * A command group that moves the intake system to the transfer position
 */
public class IntakeToTransferCommandGroup extends SequentialCommandGroup {
    /**
     * Creates a new IntakeToTransferCommandGroup
     * @param intake the intake subsystem
     */
    public IntakeToTransferCommandGroup(IntakeSubsystem intake) {
        addRequirements(intake);
        addCommands(
                intake.getClawCommand(false),
                intake.getWristCommand(180, 90, 300),
                new ParallelCommandGroup(
                        intake.getWristCommand(0, 90, 0),
                        intake.getSlidesCommand(1, 0.5)
                )
        );
    }

}
