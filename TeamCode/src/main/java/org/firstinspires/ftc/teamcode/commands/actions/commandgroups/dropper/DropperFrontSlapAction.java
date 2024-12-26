package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;

/**
 * A command group that moves the dropper slides in order to hang the specimen on the chamber forwards
 */
public class DropperFrontSlapAction extends SequentialCommandGroup {
    /**
     * Creates a new DropperFrontSlapAction
     *
     * @param dropper the dropper subsystem
     */
    public DropperFrontSlapAction(DropperSubsystem dropper) {
        addRequirements(dropper);
        addCommands(
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_FRONT_SLAP_POSITION, 0)
        );
    }
}
