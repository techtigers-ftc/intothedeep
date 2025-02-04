package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

@Config
public class IntakeTrackingAction extends CommandBase {
    private static final double TARGET_Y = 325;
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    public static double FORWARD_KP = 0.00001;
    public static double FORWARD_KI = 0.0;
    public static double FORWARD_KD = 0.0001;
    public static double FORWARD_KF = 0.001;
    public final PIDFController pidfController;
    private double tolerance;

    public IntakeTrackingAction(IntakeSubsystem intake, double tolerance, RobotState robotState) {
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
        intake.setDirectControl(true);
    }

    @Override
    public void execute() {
        double currentPosition = robotState.getBlockForwardFine();
        if (currentPosition < 0) {
            currentPosition = 0;
        }
        double currentPower = pidfController.calculate(currentPosition, TARGET_Y);
        int sign = (int) (Math.abs(currentPower) / currentPower);
        double movePower = Math.abs(currentPower) * sign;
        RobotLog.dd("tracking action", "error: %f", currentPosition - TARGET_Y);
        RobotLog.dd("tracking action", "move power: %f", movePower);
//        intake.moveSlidesRelative(moveDistance);
        intake.setMotorPower(movePower);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(robotState.getBlockForwardFine() - TARGET_Y) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        RobotLog.dd("tracking action", "ending action");
        intake.setMotorPower(0);
        intake.setDirectControl(false);
        intake.moveSlidesRelative(-2.5);
    }
}
