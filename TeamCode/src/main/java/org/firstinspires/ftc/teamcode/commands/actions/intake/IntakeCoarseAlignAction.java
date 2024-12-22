package org.firstinspires.ftc.teamcode.commands.actions.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Moves the intake slides to a position based on the coarse value obtained from the limelight
 * and rotates the claw rotation to the position absed on the orientation value from the limelight
 */
@Config
public class IntakeCoarseAlignAction extends CommandBase {
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    private final double tolerance;
    private final double rotationTolerance;
    private double targetPosition;
    private double clawTargetPosition;
    public static double SLIDES_OFFSET = 2.5;

    /**
     * Initializes the command
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     * @param tolerance  the tolerance for the target position
     */
    public IntakeCoarseAlignAction(IntakeSubsystem intake, RobotState robotState, double tolerance, double rotationTolerance) {
        this.intake = intake;
        this.robotState = robotState;
        this.tolerance = tolerance;
        this.rotationTolerance = rotationTolerance;
    }

    @Override
    public void initialize() {
        targetPosition = robotState.getBlockForwardCoarse() - SLIDES_OFFSET;
        clawTargetPosition = IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION + robotState.getBlockOrientation();
        //TODO: figure out what we want to do if the robot decides to extend too far
        intake.moveSlidesAbsolute(targetPosition);
    }

    @Override
    public void execute() {
        intake.setClawRotationAbsolute(clawTargetPosition);
        RobotLog.dd("intake coarse align", "claw rotation target:%f", 85 + robotState.getBlockOrientation());
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(targetPosition - intake.getCurrentSlidePositionInches()) < tolerance) && Math.abs(intake.getClawRotation() - clawTargetPosition) < rotationTolerance;
    }
}
