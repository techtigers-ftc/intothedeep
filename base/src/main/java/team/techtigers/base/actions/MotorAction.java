package team.techtigers.base.actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Allows a motor to run for a certain amount of time with the action interface
 */
public class MotorAction implements IAction {
    private final DcMotor motor;
    private final long duration;
    private final ElapsedTime time;
    private final double speed;

    /**
     * Initializes all values as well as throws exceptions for invalid inputs
     *
     * @param motor    DcMotor to control
     * @param speed    speed and direction motor runs
     * @param duration time for the motor to run, in milliseconds
     */
    public MotorAction(DcMotor motor, double speed, long duration) {
        if (motor == null) {
            throw new IllegalArgumentException("Null motor (arg #1)");
        }
        if (duration < 0) {
            throw new IllegalArgumentException("Duration < 0 (arg #3)");
        }
        if (speed > 1 || speed < -1) {
            throw new IllegalArgumentException("Speed not between 0 and 1 " +
                    "(arg #2)");
        }

        this.motor = motor;
        this.duration = duration;
        this.speed = speed;

        time = new ElapsedTime();
    }

    @Override
    public void start() {
        time.reset();
    }

    @Override
    public void update() {
        motor.setPower(speed);
    }

    @Override
    public boolean isFinished() {
        return time.milliseconds() >= this.duration;
    }
}