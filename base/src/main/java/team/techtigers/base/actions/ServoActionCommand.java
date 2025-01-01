package team.techtigers.base.actions;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

/**
 * Allows a servo to reach a final position in a set amount of time. This
 * allows for the synchronization of servos to reach a final position at a
 * specified time. Note that this doesn't have to be used for just a single
 * servo, and can represent a motion that is controlled by multiple servos.
 */
public abstract class ServoActionCommand extends CommandBase {
    private final long duration;
    private final ElapsedTime time;
    private final DoubleSupplier expectedPosSupplier;
    private final double INTERVAL = 30;
    private double expectedPos;
    private double linkSize;
    private double initialPos;
    private int currentLink;
    private boolean isFinished;

    /**
     * Initializes all values and throws exceptions for invalid inputs
     *
     * @param expectedPosSupplier supplier for the servo final position
     * @param duration            time for the servo to reach the final position
     */
    public ServoActionCommand(DoubleSupplier expectedPosSupplier, long duration) {
        if (duration < 0) {
            throw new IllegalArgumentException("Duration < 0 (arg #2)");
        }

        this.expectedPosSupplier = expectedPosSupplier;
        this.duration = (int) (INTERVAL * (int) (duration / INTERVAL));

        time = new ElapsedTime();
        currentLink = 1;
        isFinished = false;
        expectedPos = 0;
    }

    /**
     * Overloaded constructor that takes a target position instead of a supplier
     *
     * @param expectedPos servo final position
     * @param duration    time for the servo to reach the final position
     */
    public ServoActionCommand(double expectedPos, long duration) {
        this(() -> expectedPos, duration);
    }

    @Override
    @SuppressLint({"NewApi", "LocalSuppress"})
    public void initialize() {
        expectedPos = expectedPosSupplier.getAsDouble();
        time.reset();
        initialPos = getPosition();
        double actualDistance = expectedPos - initialPos;
        linkSize = actualDistance / (duration / INTERVAL);
        isFinished = initialPos == expectedPos;
    }

    @Override
    public void execute() {
        currentLink = (int) (time.milliseconds() / INTERVAL);

        isFinished = time.milliseconds() >= duration;
        double targetPos = isFinished ? expectedPos : initialPos + (currentLink * linkSize);
        setPosition(targetPos);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    /**
     * @return The position of the motion being controlled
     */
    protected abstract double getPosition();

    /**
     * Sets the position of the motion being controlled
     *
     * @param position The position of the motion being controlled
     */
    protected abstract void setPosition(double position);
}