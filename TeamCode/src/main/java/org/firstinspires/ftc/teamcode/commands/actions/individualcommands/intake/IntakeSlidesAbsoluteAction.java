package org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.TimeoutCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Moves the intake slides to a target position
 */
public class IntakeSlidesAbsoluteAction extends TimeoutCommand {
    private static final String LOG_TAG = IntakeSlidesAbsoluteAction.class.getSimpleName();
    private final IntakeSubsystem intake;
    private final DoubleSupplier targetPositionSupplier;
    private final double tolerance;
    private double targetPosition;

    /**
     * Initializes the command
     *
     * @param intake                 the intake subsystem
     * @param targetPositionSupplier the supplier for the target position
     * @param tolerance              the tolerance for the target position
     */
    public IntakeSlidesAbsoluteAction(IntakeSubsystem intake,
                                      DoubleSupplier targetPositionSupplier,
                                      double tolerance) {
        super(1);
        this.intake = intake;
        this.targetPositionSupplier = targetPositionSupplier;
        this.tolerance = tolerance;
        targetPosition = targetPositionSupplier.getAsDouble();
    }

    @Override
    public void initialize() {
        super.initialize();
        targetPosition = targetPositionSupplier.getAsDouble();
        intake.moveSlidesAbsolute(targetPosition);
    }

    @Override
    public void execute() {
        RobotLog.dd(LOG_TAG, "Distance to target position: %f", intake.getCurrentSlidePositionInches() - targetPosition);
        RobotLog.dd(LOG_TAG, "Current Time Elapsed: %f", getRunningTime());
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(intake.getCurrentSlidePositionInches() - targetPosition) < tolerance) || isTimeoutReached();
    }

    @Override
    public void end(boolean interrupted) {
        RobotLog.dd(LOG_TAG, "Time elapsed to run full command: %f", getRunningTime());
        if (isTimeoutReached()) {
            RobotLog.dd(LOG_TAG, "Command timed out");
        }
    }
}
