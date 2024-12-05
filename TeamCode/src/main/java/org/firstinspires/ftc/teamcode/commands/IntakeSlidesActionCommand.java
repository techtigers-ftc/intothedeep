package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import team.techtigers.base.actions.ActionCommand;

/**
 * Moves the intake slides to a target position
 */
public class IntakeSlidesActionCommand extends ActionCommand {
    private final IntakeSubsystem intake;
    private final double targetPosition;
    private final double tolerance;

    /**
     * Initializes the command
     *
     * @param intake         the intake subsystem
     * @param targetPosition the target position in inches
     * @param tolerance      the tolerance for the target position
     */
    public IntakeSlidesActionCommand(IntakeSubsystem intake, double targetPosition, double tolerance) {
        super(intake);
        this.intake = intake;
        this.targetPosition = targetPosition;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {
        intake.moveSlidesAbsolute(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(intake.getCurrentSlidePositionInches() - targetPosition) < tolerance;
    }
}
