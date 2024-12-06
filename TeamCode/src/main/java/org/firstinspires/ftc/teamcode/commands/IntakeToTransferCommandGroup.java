package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeCloseActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakePitchActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeRotationActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeSlidesActionCommand;
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
                new IntakeCloseActionCommand(intake),
                new ParallelCommandGroup(
                        new IntakePitchActionCommand(intake, 180, 300),
                        new IntakeRotationActionCommand(intake, 90, 300)
                ),
                new ParallelCommandGroup(
                        new IntakePitchActionCommand(intake, 0, 300),
                        new IntakeSlidesActionCommand(intake, 1, 0.5)
                )
        );
    }

}
