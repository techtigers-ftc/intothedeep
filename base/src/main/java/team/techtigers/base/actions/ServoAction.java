package team.techtigers.base.actions;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Allows a servo to reach a final position in a set amount of time. This
 * allows for the synchronization of servos to reach a final position at a
 * specified time
 */
public class ServoAction implements IAction {
    private final Servo servo;
    private double linkSize;
    private double initialServoPos;
    private final long duration;
    private final ElapsedTime time;
    private int currentLink;
    private final double expectedServoPos;
    private boolean isFinished;

    private final double INTERVAL = 30;

    /**
     * Initializes all values and throws exceptions for invalid inputs
     *
     * @param servo            Servo object
     * @param expectedServoPos servo final position
     * @param duration         time for the servo to reach the final position
     */
    public ServoAction(Servo servo, double expectedServoPos, long duration) {
        if (servo == null) {
            throw new IllegalArgumentException("Null servo (arg #1)");
        }
        if (duration < 0) {
            throw new IllegalArgumentException("Duration < 0 (arg #3)");
        }
        if (expectedServoPos > 1 || expectedServoPos < 0) {
            throw new IllegalArgumentException("Position not between 0 and 1" +
                    "(arg #2)");
        }

        this.expectedServoPos = expectedServoPos;
        this.servo = servo;
        this.duration = (int) (INTERVAL * (int) (duration / INTERVAL));

        time = new ElapsedTime();
        currentLink = 1;
        isFinished = false;
    }

    @Override
    public void start() {
        time.reset();
        initialServoPos = servo.getPosition();
        double actualDistance = expectedServoPos - initialServoPos;
        linkSize = actualDistance / (duration / INTERVAL);
        isFinished = initialServoPos == expectedServoPos;
    }

    @Override
    public void update() {
        currentLink = (int) (time.milliseconds() / INTERVAL);

        isFinished = time.milliseconds() >= duration;
        double targetPos = isFinished ? expectedServoPos : initialServoPos + (currentLink * linkSize);
        servo.setPosition(targetPos);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}