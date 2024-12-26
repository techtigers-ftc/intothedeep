package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;

/**
 * A command group that moves the dropper slides in order to hang the specimen on the chamber
 */
public class HangSpecimenAction extends SequentialCommandGroup {
    /**
     * Creates a new HangSpecimenAction
     *
     * @param dropper the dropper subsystem
     */
    public HangSpecimenAction(DropperSubsystem dropper) {
        addRequirements(dropper);
        addCommands(
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_BACK_SLAP_POSITION, 0)
        );
    }
}
