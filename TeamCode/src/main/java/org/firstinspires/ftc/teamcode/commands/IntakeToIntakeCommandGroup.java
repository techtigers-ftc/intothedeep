package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

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
                intake.getClawCommand(true),
                intake.getSlidesCommand(20, 0.5),
                intake.getWristCommand(180, 90, 300)
        );
    }
}
