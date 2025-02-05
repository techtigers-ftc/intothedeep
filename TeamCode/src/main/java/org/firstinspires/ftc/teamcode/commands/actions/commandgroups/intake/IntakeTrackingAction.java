package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.TimeoutCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Command to move the slides until the small camera sees the block is in the right place
 */
@Config
public class IntakeTrackingAction extends TimeoutCommand {
    private static final double TARGET_Y = 450;
    public static double FORWARD_KP = 0.001;
    public static double FORWARD_KI = 0.0;
    public static double FORWARD_KD = 0;
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
        super(3);
        this.intake = intake;
        this.tolerance = tolerance;
        this.robotState = robotState;
        pidfController = new PIDFController(FORWARD_KP,
                FORWARD_KI,
                FORWARD_KD,
                FORWARD_KF);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        intake.setDirectControl(true);
    }

    @Override
    public void execute() {
        double currentPosition = robotState.getBlockForwardFine();
        if (currentPosition < 0) {
            currentPosition = 0;
        }
        double movePower = pidfController.calculate(currentPosition, TARGET_Y);
        RobotLog.dd("tracking action", "error: %f", currentPosition - TARGET_Y);
        RobotLog.dd("tracking action", "move power: %f", movePower);
        intake.setMotorPower(movePower);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(robotState.getBlockForwardFine() - TARGET_Y) < tolerance) || isTimeoutReached();
    }

    @Override
    public void end(boolean interrupted) {
        RobotLog.dd("tracking action", "ending tracking action");
        robotState.setDetectedFineBlockOrientation(robotState.getBlockOrientation());
        intake.setMotorPower(0);
        intake.setDirectControl(false);
        intake.moveSlidesRelative(-3.5);
    }
}
