package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.TimeoutCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

@Config
public class IntakeTrackingAction extends TimeoutCommand {
    private static final double TARGET_Y = 450;
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    public static double FORWARD_KP = 0.001;
    public static double FORWARD_KI = 0.0;
    public static double FORWARD_KD = 0;
    public static double FORWARD_KF = 0;
    private static final double PIXELS_PER_INCH = 88.27586207;
    public final PIDFController pidfController;
    private double tolerance;

    public IntakeTrackingAction(IntakeSubsystem intake, double tolerance, RobotState robotState) {
        super(2);
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
        double currentPower = pidfController.calculate(currentPosition, TARGET_Y);
        double movePower = currentPower * robotState.getVoltage() / 12.0;
        movePower = Range.clip(movePower, -1, 1);
        RobotLog.dd("tracking action", "error: %f", currentPosition - TARGET_Y);
        RobotLog.dd("tracking action", "move power: %f", movePower);
//        intake.moveSlidesRelative(moveDistance);
        intake.setMotorPower(movePower);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(robotState.getBlockForwardFine() - TARGET_Y) < tolerance) || isTimeoutReached();
    }

    @Override
    public void end(boolean interrupted) {
        RobotLog.dd("tracking action", "ending action");
        RobotLog.dd("tracking action", "ending error: %f", -(robotState.getBlockForwardFine() - TARGET_Y) / PIXELS_PER_INCH);
        intake.setMotorPower(0);
        intake.setDirectControl(false);
//        intake.moveSlidesRelative(-(robotState.getBlockForwardFine() - TARGET_Y) / PIXELS_PER_INCH - 3.5);
        intake.moveSlidesRelative(-3.5);
    }
}
