package org.firstinspires.ftc.teamcode.commands.actions.dropper;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;

public class DropperCloseActionCommand extends CommandBase {
    private final DropperSubsystem dropper;
    private final long waitTime;
    private final ElapsedTime time;

    /**
     * Initializes the command
     *
     * @param dropper  the dropper subsystem
     * @param waitTime the time to wait before the command is finished in milliseconds
     */
    public DropperCloseActionCommand(DropperSubsystem dropper, long waitTime) {
        this.dropper = dropper;
        this.waitTime = waitTime;
        time = new ElapsedTime();
    }

    /**
     * Initializes the command with a wait time of 0
     *
     * @param dropper the dropper subsystem
     */
    public DropperCloseActionCommand(DropperSubsystem dropper) {
        this(dropper, 0);
    }

    @Override
    public void initialize() {
        dropper.closeClaw();
        time.reset();
    }

    @Override
    public boolean isFinished() {
        return time.milliseconds() >= waitTime;
    }
}
