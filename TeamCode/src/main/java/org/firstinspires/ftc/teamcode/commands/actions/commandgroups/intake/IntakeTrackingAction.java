package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.commands.TimeoutCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

/**
 * Command to move the slides until the small camera sees the block is in the right place
 */
@Config
public class IntakeTrackingAction extends TimeoutCommand {
    private static final double TARGET_Y = 0;
    public static double FORWARD_KP = 0.2;
    public static double FORWARD_KI = 0;
    public static double FORWARD_KD = 0.01;
    public static double FORWARD_KF = 0;
    public final PIDFController pidfController;
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    private double tolerance;

    /**
     * Constructs a new IntakeTrackingAction
     *
     * @param intake     the intake subsystem
     * @param tolerance  the tolerance for the command (in pixels)
     * @param robotState robot state
     */
    public IntakeTrackingAction(IntakeSubsystem intake, double tolerance, RobotState robotState) {
        super(10);
        addRequirements(intake);
        this.intake = intake;
        this.tolerance = tolerance;
        this.robotState = robotState;
        pidfController = new PIDFController(FORWARD_KP,
                FORWARD_KI,
                FORWARD_KD,
                FORWARD_KF);
    }

    @Override
    public void initialize() {
        super.initialize();
        intake.setDirectControl(true);
    }

    @Override
    public void execute() {
//        double currentPosition = robotState.getBlockForwardFine();
//        if (robotState.getFineBlockDetectionState() == BlockDetectionState.NOT_DETECTED) {
//            currentPosition = -2.5;
//        }
//        double movePower = pidfController.calculate(currentPosition, TARGET_Y);
        intake.setMotorPower(0.35);
    }

    @Override
    public boolean isFinished() {
        return //((Math.abs(robotState.getBlockForwardFine() - TARGET_Y) <
        // tolerance)
                robotState.getFineBlockDetectionState() == BlockDetectionState.DETECTED || isTimeoutReached();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotorPower(0);
        intake.setDirectControl(false);
        intake.moveSlidesRelative(0);
    }
}
