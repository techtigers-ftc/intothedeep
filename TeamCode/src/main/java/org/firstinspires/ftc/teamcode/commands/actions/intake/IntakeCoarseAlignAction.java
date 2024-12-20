package org.firstinspires.ftc.teamcode.commands.actions.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Moves the intake slides to a position based on the coarse value obtained from the limelight
 * and rotates the claw rotation to the position absed on the orientation value from the limelight
 */
public class IntakeCoarseAlignAction extends CommandBase {
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    private final double tolerance;
    private double targetPosition;

    /**
     * Initializes the command
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     * @param tolerance  the tolerance for the target position
     */
    public IntakeCoarseAlignAction(IntakeSubsystem intake, RobotState robotState, double tolerance) {
        this.intake = intake;
        this.robotState = robotState;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {
        targetPosition = robotState.getBlockForwardCoarse();
        //TODO: figure out what we want to do if the robot decides to extend too far
        intake.moveSlidesAbsolute(targetPosition);
        intake.setClawRotationAbsolute(85 - robotState.getBlockOrientation());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetPosition - intake.getCurrentSlidePositionInches()) < tolerance;
    }
}
