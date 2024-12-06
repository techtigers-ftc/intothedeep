package org.firstinspires.ftc.teamcode.commands.actions.dropper;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;

/**
 * Moves the dropper slides to a target position
 */
public class DropperSlidesActionCommand extends CommandBase {
    private final DropperSubsystem dropper;
    private final double targetPosition;
    private final double tolerance;

    /**
     * Initializes the command
     *
     * @param dropper        the dropper subsystem
     * @param targetPosition the target position in inches
     * @param tolerance      the tolerance for the target position
     */
    public DropperSlidesActionCommand(DropperSubsystem dropper, double targetPosition, double tolerance) {
        this.dropper = dropper;
        this.targetPosition = targetPosition;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {
        dropper.moveSlidesAbsolute(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(dropper.getCurrentSlidePositionInches() - targetPosition) < tolerance;
    }
}
