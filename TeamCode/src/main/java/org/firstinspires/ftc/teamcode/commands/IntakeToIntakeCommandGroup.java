package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.IntakeOpenActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.IntakePitchActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.IntakeRotationActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.IntakeSlidesActionCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * A command group that moves the intake system to the intake position
 */
public class IntakeToIntakeCommandGroup extends SequentialCommandGroup {
    /**
     * Creates a new IntakeToIntakeCommandGroup
     *
     * @param intake the intake subsystem
     */
    public IntakeToIntakeCommandGroup(IntakeSubsystem intake) {
        addRequirements(intake);
        addCommands(
                new IntakeOpenActionCommand(intake),
                new ParallelCommandGroup(
                        new IntakeSlidesActionCommand(intake, 20, 0.5),
                        new IntakeRotationActionCommand(intake, 90, 300)
                ),
                new IntakePitchActionCommand(intake, 180, 300)
        );
    }
}
