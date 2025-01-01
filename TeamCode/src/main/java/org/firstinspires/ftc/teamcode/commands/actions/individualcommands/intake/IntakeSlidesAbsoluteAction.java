package org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Moves the intake slides to a target position
 */
public class IntakeSlidesAbsoluteAction extends CommandBase {
    private final IntakeSubsystem intake;
    private final DoubleSupplier targetPositionSupplier;
    private final double tolerance;
    private double targetPosition;

    /**
     * Initializes the command
     *
     * @param intake         the intake subsystem
     * @param targetPositionSupplier the supplier for the target position
     * @param tolerance      the tolerance for the target position
     */
    public IntakeSlidesAbsoluteAction(IntakeSubsystem intake,
                                      DoubleSupplier targetPositionSupplier,
                                      double tolerance) {
        this.intake = intake;
        this.targetPositionSupplier = targetPositionSupplier;
        this.tolerance = tolerance;
        targetPosition = targetPositionSupplier.getAsDouble();
    }

    @Override
    public void initialize() {
        targetPosition = targetPositionSupplier.getAsDouble();
        intake.moveSlidesAbsolute(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(intake.getCurrentSlidePositionInches() - targetPosition) < tolerance;
    }
}
