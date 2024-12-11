package org.firstinspires.ftc.teamcode.commands.actions.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

/**
 * Moves the intake slides to a position inside the claw based on the values from the camera
 * mounted on the claw
 */
public class IntakeFineCameraAction extends CommandBase {
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    private final double pixelTolerance;
    private final static double INCHES_PER_PIXEL = 11 / 1280.0;
    private final static double X_POSITION = 700;

    /**
     * Initializes the command
     *
     * @param intake         the intake subsystem
     * @param robotState     the robot state
     * @param pixelTolerance      the tolerance for the target position
     */
    public IntakeFineCameraAction(IntakeSubsystem intake, RobotState robotState, double pixelTolerance) {
        this.intake = intake;
        this.robotState = robotState;
        this.pixelTolerance = pixelTolerance;
    }

    @Override
    public void execute() {
        double positionChange;
        if(robotState.getBlockDetectionState() == BlockDetectionState.NOT_DETECTED) {
                positionChange = 0.5;
        } else {
            positionChange = (X_POSITION - robotState.getBlockLateralFine()) * INCHES_PER_PIXEL;
            intake.setRotationAbsolute(90 - robotState.getBlockOrientation());
        }
        intake.moveSlidesRelative(positionChange);
        RobotLog.dd("fine action", "moving slides");
        RobotLog.dd("fine action", "target movement amount: %f", positionChange);
    }

    @Override
    public void end(boolean interrupted) {
        RobotLog.dd("fine action", "complete");
    }

    @Override
    public boolean isFinished() {
        return X_POSITION - robotState.getBlockLateralFine() < pixelTolerance;
    }
}
