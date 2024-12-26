package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;

/**
 * A command group that moves the dropper slides in order to hang the specimen on the chamber backwards
 */
public class DropperBackSlapAction extends SequentialCommandGroup {
    /**
     * Creates a new DropperBackSlapAction
     * @param dropper the dropper subsystem
     */
    public DropperBackSlapAction(DropperSubsystem dropper) {
        addRequirements(dropper);
        addCommands(
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_FRONT_SLAP_POSITION, 0)
        );
    }
}
