package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem that controls the ascent mechanism.
 */
public class AscentSubsystem extends CloseableSubsystem {
    public static final double ASCENT_INITIAL_HEIGHT = 12;
    public static final double JACKS_DISENGAGE_HEIGHT = 9;
    private final Servo changingTransmission;
    private final Servo leftJackServo;
    private final Servo rightJackServo;
    private final RobotState robotState;
    private boolean jacksEngaged;

    /**
     * Constructs a new AscentSubsystem.
     *
     * @param hardwareMap The hardware map, used to get hardware references
     * @param robotState  The state of the robot
     */
    public AscentSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        changingTransmission = hardwareMap.get(Servo.class,
                "transmission_switch");
        leftJackServo = hardwareMap.get(Servo.class, "left_jack");
        rightJackServo = hardwareMap.get(Servo.class, "right_jack");

        leftJackServo.setDirection(Servo.Direction.REVERSE);

        jacksEngaged = false;

        changingTransmission.setPosition(0.5);
        robotState.setIsAscending(false);
        disengageJacks();
    }

    /**
     * Disengages the jacks that lift the robot
     */
    public void disengageJacks() {
        leftJackServo.setPosition(0);
        rightJackServo.setPosition(0);
        jacksEngaged = false;
    }

    /**
     * Engages the jacks that lift the robot
     */
    public void engageJacks() {
        leftJackServo.setPosition(0.99);
        rightJackServo.setPosition(0.99);
        jacksEngaged = true;
    }

    /**
     * Engages the switching transmission and the jacks
     */
    public void engageAscent() {
        changingTransmission.setPosition(0.69);
        robotState.setIsAscending(true);
        engageJacks();
    }

    /**
     * Gets if the jacks are engaged
     *
     * @return if the jacks are engaged
     */
    public boolean areJacksEngaged() {
        return jacksEngaged;
    }

    /**
     * Gets the position of the changing transmission servo. Used for testing
     *
     * @return the position of the changing transmission servo
     */
    public double getChangingTransmissionPosition() {
        return changingTransmission.getPosition();
    }

    /**
     * Sets the position of the changing transmission servo. Used for testing
     *
     * @param position the position to set the servo to
     */
    public void setChangingTransmissionPosition(double position) {
        changingTransmission.setPosition(position);
    }

    @Override
    public void close() {
        disengageJacks();
    }
}
