package org.firstinspires.ftc.teamcode.commands.actions.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Moves the intake slides to a target position based on the coarse limelight values
 */
public class IntakeSlidesCoarseAlignAction extends CommandBase {
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    private final double targetPosition;
    private final double tolerance;

    /**
     * Initializes the command
     *
     * @param intake         the intake subsystem
     * @param targetPosition the target position in inches
     * @param tolerance      the tolerance for the target position
     */
    public IntakeSlidesCoarseAlignAction(IntakeSubsystem intake, double targetPosition, double tolerance) {
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
