package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem that controls the ascent mechanism.
 */
public class AscentSubsystem extends CloseableSubsystem {
    public static final double ASCENT_SLIDES_INITIAL_HEIGHT = 12;
    public static final double JACKS_SLIDES_DISENGAGE_HEIGHT = 9;
    public static final double ASCENT_UNENGAGED_POSITION = 0.5;
    public static final double ASCENT_ENGAGED_POSITION = 0.71;
    public static final double JACKS_UNENGAGED_POSITION = 0;
    public static final double JACKS_ENGAGED_POSITION = 0.84;
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

        rightJackServo.setDirection(Servo.Direction.REVERSE);

        jacksEngaged = false;

        changingTransmission.setPosition(ASCENT_UNENGAGED_POSITION);
        robotState.setIsAscending(false);
        disengageJacks();
    }

    /**
     * Disengages the jacks that lift the robot
     */
    public void disengageJacks() {
        leftJackServo.setPosition(JACKS_UNENGAGED_POSITION);
        rightJackServo.setPosition(JACKS_UNENGAGED_POSITION);
        jacksEngaged = false;
    }

    /**
     * Engages the jacks that lift the robot
     */
    public void engageJacks() {
        leftJackServo.setPosition(JACKS_ENGAGED_POSITION);
        rightJackServo.setPosition(JACKS_ENGAGED_POSITION);
        jacksEngaged = true;
    }

    /**
     * Engages the switching transmission and the jacks
     */
    public void engageAscent() {
        changingTransmission.setPosition(ASCENT_ENGAGED_POSITION);
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
