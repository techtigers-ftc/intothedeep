package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperSlidesRelativeAction;
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
                new DropperSlidesRelativeAction(dropper, 3, 0.25),
                new DropperOpenAction(dropper)
        );
    }
}
