package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

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
     * @return if timeout is valid and is exceeded returns true
     */
    protected final boolean isTimeoutReached(){
        return timeout > 0 && timer.seconds() > timeout;
    }
}
