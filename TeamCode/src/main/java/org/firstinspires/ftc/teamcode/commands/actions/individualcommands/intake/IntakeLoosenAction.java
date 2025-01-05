package org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Action Command to loosen the intake claw
 */
public class IntakeLoosenAction extends CommandBase {
    private final IntakeSubsystem intake;
    private final long waitTime;
    private final ElapsedTime time;

    /**
     * Initializes the command
     *
     * @param intake the intake subsystem
     * @param waitTime the time to wait before the command is finished in milliseconds
     */
    public IntakeLoosenAction(IntakeSubsystem intake, long waitTime) {
        this.intake = intake;
        this.waitTime = waitTime;
        time = new ElapsedTime();
    }

    /**
     * Initializes the command with a wait time of 0
     *
     * @param intake the intake subsystem
     */
    public IntakeLoosenAction(IntakeSubsystem intake) {
        this(intake, 0);
    }

    @Override
    public void initialize() {
        intake.loosenClaw();
        time.reset();
    }

    @Override
    public boolean isFinished() {
        return time.milliseconds() >= waitTime;
    }
}
