package org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.TimeoutCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * An action which moves the intakes at a raw power for a certain amount of time
 */
public class IntakeRawPowerAction extends TimeoutCommand {
    private final IntakeSubsystem intake;
    private double power;

    /**
     * Constructor for IntakeRawPowerAction
     *
     * @param power the power to drive the robot at (-1 to 1)
     * @param time  the time to drive the robot for
     */
    public IntakeRawPowerAction(IntakeSubsystem intake, double power, double time) {
        super(time);
        this.intake = intake;
        this.power = power;
    }

    @Override
    public void initialize() {
        super.initialize();
        intake.setDirectControl(true);
    }

    @Override
    public void execute() {
        intake.setMotorPower(power);
    }

    @Override
    public boolean isFinished() {
        return isTimeoutReached();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotorPower(0);
        intake.setDirectControl(false);
        intake.moveSlidesRelative(0);
    }
}
