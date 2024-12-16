package org.firstinspires.ftc.teamcode.commands.actions.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

/**
 * Moves the intake slides to a position inside the claw based on the values from the camera
 * mounted on the claw
 */
@Config
public class IntakeFineCameraAction extends CommandBase {
    private final static double INCHES_PER_PIXEL = 11 / 1280.0;
    private final static double X_POSITION = 700;
    public static double SLIDES_INCREMENT = 0.5;
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    private final double pixelTolerance;

    /**
     * Initializes the command
     *
     * @param intake         the intake subsystem
     * @param robotState     the robot state
     * @param pixelTolerance the tolerance for the target position
     */
    public IntakeFineCameraAction(IntakeSubsystem intake, RobotState robotState, double pixelTolerance) {
        this.intake = intake;
        this.robotState = robotState;
        this.pixelTolerance = pixelTolerance;
    }

    @Override
    public void execute() {
        if (robotState.getBlockDetectionState() == BlockDetectionState.NOT_DETECTED) {
//            intake.moveSlidesRelative(SLIDES_INCREMENT);
//            RobotLog.dd("fine action", "target movement amount: %f", SLIDES_INCREMENT);
        } else {
//            double positionChange = robotState.getBlockForwardFine() * INCHES_PER_PIXEL;
//            intake.moveSlidesRelative(X_POSITION - positionChange);
//            RobotLog.dd("fine action", "target movement amount: %f", (X_POSITION - positionChange));


//            intake.moveSlidesRelative(0);
            intake.setClawRotationAbsolute(robotState.getBlockOrientation());
            // TODO: Fix the INCHES_PER_PIXEL using trig and uncomment above code
//            intake.setRotationAbsolute(90 - robotState.getBlockOrientation());
        }
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
