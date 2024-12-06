package team.techtigers.base.actions;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Allows a motor to run for a certain amount of time with the action
 * command interface
 */
public abstract class MotorActionCommand extends CommandBase {
    private final long duration;
    private final ElapsedTime time;
    private final double speed;

    /**
     * Initializes all values as well as throws exceptions for invalid inputs
     *
     * @param speed    speed and direction motor runs
     * @param duration time for the motor to run, in milliseconds
     */
    public MotorActionCommand(double speed, long duration) {
        if (duration < 0) {
            throw new IllegalArgumentException("Duration < 0 (arg #3)");
        }
        if (speed > 1 || speed < -1) {
            throw new IllegalArgumentException("Speed not between 0 and 1 " +
                    "(arg #2)");
        }

        this.duration = duration;
        this.speed = speed;

        time = new ElapsedTime();
    }

    @Override
    public void initialize() {
        time.reset();
        setPower(speed);
    }

    @Override
    public boolean isFinished() {
        return time.milliseconds() >= this.duration;
    }

    /**
     * Sets the motor to run at a certain power
     *
     * @param power power to set the motor to
     */
    protected abstract void setPower(double power);
}