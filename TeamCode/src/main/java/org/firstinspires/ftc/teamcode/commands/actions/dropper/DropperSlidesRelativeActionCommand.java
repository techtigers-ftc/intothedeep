package org.firstinspires.ftc.teamcode.commands.actions.dropper;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;

/**
 * Moves the dropper slides to a target position relative to the current position of the slides
 */
public class DropperSlidesRelativeActionCommand extends DropperSlidesAbsoluteActionCommand {
    /**
     * Initializes the command
     *
     * @param dropper        the dropper subsystem
     * @param targetPosition the change in target position in inches
     * @param tolerance      the tolerance for the target position
     */
    public DropperSlidesRelativeActionCommand(DropperSubsystem dropper, double targetPosition, double tolerance) {
        super(dropper, targetPosition + dropper.getCurrentSlidePositionInches(), tolerance);
    }
}
