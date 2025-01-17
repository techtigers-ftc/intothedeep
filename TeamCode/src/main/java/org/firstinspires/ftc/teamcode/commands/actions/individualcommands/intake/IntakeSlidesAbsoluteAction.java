package org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

import java.util.function.DoubleSupplier;

/**
 * Moves the intake slides to a target position
 */
public class IntakeSlidesAbsoluteAction extends CommandBase {
    private static final String LOG_TAG = IntakeSlidesAbsoluteAction.class.getSimpleName();
    private final IntakeSubsystem intake;
    private final RobotState robotState;
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
    public IntakeSlidesAbsoluteAction(IntakeSubsystem intake, RobotState robotState,
                                      DoubleSupplier targetPositionSupplier,
                                      double tolerance) {
        this.intake = intake;
        this.targetPositionSupplier = targetPositionSupplier;
        this.tolerance = tolerance;
        this.robotState = robotState;
        targetPosition = targetPositionSupplier.getAsDouble();
    }

    @Override
    public void initialize() {
        targetPosition = targetPositionSupplier.getAsDouble();
        if(targetPosition > IntakeSubsystem.SLIDES_MAX) {
            RobotLog.ww(LOG_TAG, "Extending Slides Too Far: %s", targetPosition);
            targetPosition = IntakeSubsystem.SLIDES_MAX;
            robotState.setError(RobotError.EXTENDING_SLIDES_TOO_FAR);
        }
        RobotLog.dd(LOG_TAG, "Target pos: %f", targetPosition);
        intake.moveSlidesAbsolute(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(intake.getCurrentSlidePositionInches() - targetPosition) < tolerance;
    }
}
