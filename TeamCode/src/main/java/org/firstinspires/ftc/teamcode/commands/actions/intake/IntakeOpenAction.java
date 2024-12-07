package org.firstinspires.ftc.teamcode.commands.actions.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Action Command to open the intake claw
 */
public class IntakeOpenAction extends CommandBase {
    private final IntakeSubsystem intake;
    private final long waitTime;
    private final ElapsedTime time;

    /**
     * Initializes the command
     *
     * @param intake the intake subsystem
     * @param waitTime the time to wait before the command is finished in milliseconds
     */
    public IntakeOpenAction(IntakeSubsystem intake, long waitTime) {
        this.intake = intake;
        this.waitTime = waitTime;
        time = new ElapsedTime();
    }

    /**
     * Initializes the command with a wait time of 0
     *
     * @param intake the intake subsystem
     */
    public IntakeOpenAction(IntakeSubsystem intake) {
        this(intake, 0);
    }

    @Override
    public void initialize() {
        intake.openClaw();
        time.reset();
    }

    @Override
    public boolean isFinished() {
        return time.milliseconds() >= waitTime;
    }
}
