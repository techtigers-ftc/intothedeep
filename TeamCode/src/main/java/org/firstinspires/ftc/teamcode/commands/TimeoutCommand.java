package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A command which times out once the command has reached past the timeout
 */
public class TimeoutCommand extends CommandBase {
    private double timeout;
    private ElapsedTime timer;

    /**
     * Constructs a new timeout command
     *
     * @param timeout the amount of time for the timeout
     */
    public TimeoutCommand(double timeout) {
        this.timeout = timeout;
        timer = new ElapsedTime();
    }

    @Override
    public void initialize(){
        timer.reset();
    }

    /**
     * @return the amount of time that the command has been running for
     */
    protected final double getRunningTime() {
        return timer.seconds();
    }

    /**
     * @return the amount of time that the command has remaining until the timeout is reached
     */
    protected final double getRemainingTime() {
        return timeout - timer.seconds();
    }

    /**
     * @return if timeout is valid and is exceeded returns true
     */
    protected final boolean isTimeoutReached(){
        return timeout > 0 && timer.seconds() > timeout;
    }
}
