package org.firstinspires.ftc.teamcode.commands.actions.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

/**
 * Moves the intake slides to a position based on the coarse value obtained from the limelight
 */
public class IntakeSlidesCoarseAlignAction extends CommandBase {
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    private final double tolerance;
    private double targetPosition;

    /**
     * Initializes the command
     *
     * @param intake         the intake subsystem
     * @param robotState     the robot state
     * @param tolerance      the tolerance for the target position
     */
    public IntakeSlidesCoarseAlignAction(IntakeSubsystem intake, RobotState robotState, double tolerance) {
        this.intake = intake;
        this.robotState = robotState;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {
        targetPosition = robotState.getBlockForwardCoarse();
        intake.moveSlidesAbsolute(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return  Math.abs(targetPosition - intake.getCurrentSlidePositionInches()) < tolerance;
    }
}
