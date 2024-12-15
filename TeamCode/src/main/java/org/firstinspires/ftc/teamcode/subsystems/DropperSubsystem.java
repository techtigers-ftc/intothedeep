package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.SlideController;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;

import team.techtigers.base.CloseableSubsystem;

/**
 * Encapsulates all hardware, methods, and attributes of the dropper subsystem, including the
 * vertical slides, arm, and the claw.
 */
@Config
public class DropperSubsystem extends CloseableSubsystem {
    private static final double SPOOL_CIRCUMFERENCE_INCHES = 1.27 * Math.PI;
    private static final double SPOOL_GEAR_RATIO = 1.0; // Driver / Follower
    private static final double TICKS_PER_ROTATION = 384.5;
    private static final double ERROR_FACTOR = 29.0 / 25.2 * 0.97;
    private static final double INCHES_PER_MOTOR_TICK = ERROR_FACTOR * (SPOOL_GEAR_RATIO * SPOOL_CIRCUMFERENCE_INCHES) / TICKS_PER_ROTATION;
    private static final double TICKS_PER_INCHES = 1 / INCHES_PER_MOTOR_TICK;
    private static final double PITCH_GEAR_RATIO = 1.0; // Driver / Follower
    private static final double ROTATION_GEAR_RATIO = 1.0; // Driver / Follower
    private static final double DROPPER_PITCH_RANGE = 355;
    private static final double DROPPER_ROTATION_RANGE = 180;
    private static final double DROPPER_ROTATION_BUFFER = 0;
    public static double CLAW_OPENED_POSITION = 0;
    public static double CLAW_CLOSED_POSITION = 0.75;
    public static double KP = 0.015;
    public static double KI = 0;
    public static double KD = 0.000000001;
    public static double KF = 0;
    public static double SLIDES_TOLERANCE = 1;
    private final DcMotor rightSlideMotor;
    private final DcMotor leftSlideMotor;
    private final DcMotor encoderMotor;
    private final DcMotorEx currentMotor;
    private final Servo rightPitchServo;
    private final Servo leftPitchServo;
    private final Servo rotationServo;
    private final Servo grabServo;
    private final RobotState robotState;
    private final SlideController slideController;

    /**
     * Initializes dropper subsystem
     *
     * @param hardwareMap: is a variable where you configure all the devices in the specific subsystem
     */
    public DropperSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_dropper_slide");
        leftSlideMotor = hardwareMap.get(DcMotor.class, "left_dropper_slide");
        // Pitch servo zero is all the way around, up against the bar
        rightPitchServo = hardwareMap.get(Servo.class, "right_dropper_pitch");
        leftPitchServo = hardwareMap.get(Servo.class, "left_dropper_pitch");
        // Rotation servo zero is directly in the transfer position
        rotationServo = hardwareMap.get(Servo.class, "dropper_rotation");
        // Grab servo zero is at the open position for the claw
        grabServo = hardwareMap.get(Servo.class, "dropper_claw");

        PIDFCoefficients forwardPIDF = new PIDFCoefficients(KP, KI, KD, KF);
        slideController = new SlideController(TICKS_PER_INCHES, forwardPIDF);

        grabServo.setDirection(Servo.Direction.REVERSE);

        leftPitchServo.setDirection(Servo.Direction.FORWARD);
        rightPitchServo.setDirection(Servo.Direction.REVERSE);

        leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        encoderMotor = rightSlideMotor; // Assuming rightSlideMotor is the encoder motor
        currentMotor = (DcMotorEx) encoderMotor;
        resetSlides();

        slideController.setTolerance(SLIDES_TOLERANCE);

        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setPitchAbsolute(195);
        setRotationAbsolute(0);
        openClaw();
    }

    /**
     * Resets encoder values of the slide motors
     */
    public void resetSlides() {
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        moveSlidesAbsolute(0);
    }

    /**
     * Method that moves servo to make the claw open
     */
    public void openClaw() {
        grabServo.setPosition(CLAW_OPENED_POSITION);
        robotState.setDropperClawState(ClawState.OPEN);
    }

    /**
     * Method that moves servo to make the claw close
     */
    public void closeClaw() {
        grabServo.setPosition(CLAW_CLOSED_POSITION);
        robotState.setDropperClawState(ClawState.CLOSED);
    }

    /**
     * Toggles the claw between open and closed
     */
    public void toggleClaw() {
        if(robotState.getDropperClawState() == ClawState.CLOSED) {
            openClaw();
        } else {
            closeClaw();
        }
    }

    /**
     * Stops the slides wherever it's currently at
     */
    public void stopSlides() {
        rightSlideMotor.setPower(0);
        leftSlideMotor.setPower(0);
    }

    /**
     * Gets the current position of the slides in inches
     *
     * @return the current position of the slides in inches
     */
    public double getCurrentSlidePositionInches() {
        return encoderMotor.getCurrentPosition() * INCHES_PER_MOTOR_TICK;
    }

    /**
     * Gets the target position of the slides in inches
     *
     * @return the target position of the slides in inches
     */
    public double getTargetPositionInches() {
        return slideController.targetTicks * INCHES_PER_MOTOR_TICK;
    }

    /**
     * Increments slides from wherever it is currently
     *
     * @param position Amount you are incrementing by inches
     */
    public void moveSlidesRelative(double position) {
        if ((getCurrentSlidePositionInches() + position) < 0){
            slideController.moveToInches(0);
        } else {
            slideController.moveToInches(getCurrentSlidePositionInches() + position);
        }
    }

    /**
     * Moves slides to that position from wherever it is
     *
     * @param position Position where you want to set the slides to in inches
     */
    public void moveSlidesAbsolute(double position) {
        if (position < 0){
            slideController.moveToInches(0);
        } else{
            slideController.moveToInches(position);
        }
    }

    /**
     * Method that increments the wrist from where it is currently at
     *
     * @param pitch    Amount you want to increment by for the dropper in degrees
     * @param rotation Amount you want to increment by for the dropper in degrees
     */
    public void setWristRelative(double pitch, double rotation) {
        setWristAbsolute(getPitch() + pitch, getRotation() + rotation);
    }

    /**
     * Sets the wrist to an absolute position in degrees
     *
     * @param pitch    The angle to set the pitch to in degrees
     * @param rotation The angle to set the rotation to in degrees
     */
    public void setWristAbsolute(double pitch, double rotation) {
        rightPitchServo.setPosition(pitch * PITCH_GEAR_RATIO / DROPPER_PITCH_RANGE);
        leftPitchServo.setPosition(pitch * PITCH_GEAR_RATIO / DROPPER_PITCH_RANGE);
        rotationServo.setPosition((rotation + DROPPER_ROTATION_BUFFER) * ROTATION_GEAR_RATIO / DROPPER_ROTATION_RANGE);
        robotState.setDropperClawPitch(getPitch());
        robotState.setDropperClawRotation(getRotation());
    }

    /**
     * @return the pitch of the dropper arm in degrees
     */
    public double getPitch() {
        return DROPPER_PITCH_RANGE * leftPitchServo.getPosition();
    }

    /**
     * @return the rotation of the claw in degrees
     */
    public double getRotation() {
        return DROPPER_ROTATION_RANGE * rotationServo.getPosition() - DROPPER_ROTATION_BUFFER;
    }

    /**
     * Sets the pitch of the wrist, while keeping the rotation the same
     *
     * @param pitchAngle the desired pitch of the wrist
     */
    public void setPitchAbsolute(double pitchAngle) {
        setWristAbsolute(pitchAngle, getRotation());
    }

    /**
     * Sets the pitch angle relative to the current angle
     *
     * @param pitchAngle the desired change in pitch in degrees
     */
    public void setPitchRelative(double pitchAngle) {
        setWristRelative(pitchAngle, 0);
    }

    /**
     * Sets the rotation of the wrist, while keeping the pitch the same
     *
     * @param rotationAngle the desired rotation of the wrist
     */
    public void setRotationAbsolute(double rotationAngle) {
        setWristAbsolute(getPitch(), rotationAngle);
    }

    /**
     * @return the current draw of the slide motors
     */
    public double getSlideCurrent() {
        return currentMotor.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Sets the rotation of the wrist, while keeping the pitch the same
     *
     * @param rotationAngle the desired change in pitch of the wrist
     */
    public void setRotationRelative(double rotationAngle) {
        setWristRelative(0, rotationAngle);
    }

    public void setSlidesPower(double power) {
        rightSlideMotor.setPower(power);
        leftSlideMotor.setPower(power);
    }

    @Override
    public void periodic() {
        double power = slideController.calculateMotorPowers(encoderMotor.getCurrentPosition());
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);

        if(getSlideCurrent() > 3.5){
            moveSlidesRelative(0);
        }

        robotState.setVerticalExtended(encoderMotor.getCurrentPosition() > 100);

        RobotLog.dd(tag, "Current: %f Target %f",
                getCurrentSlidePositionInches(), getTargetPositionInches());
    }
}
