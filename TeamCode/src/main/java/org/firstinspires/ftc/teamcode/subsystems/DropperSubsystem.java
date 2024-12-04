package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.SlideController;

import team.techtigers.base.CloseableSubsytem;

@Config
public class DropperSubsystem extends CloseableSubsytem {
    private final DcMotor rightSlideMotor;
    private final DcMotor leftSlideMotor;
    private final DcMotor encoderMotor;
    private final Servo rightPitchServo;
    private final Servo leftPitchServo;
    private final Servo rotationServo;
    private final Servo grabServo;
    private final RobotState robotState;
    private double targetPos;

    private static final double SPOOL_CIRCUMFERENCE_INCHES = 1.27 * Math.PI;
    private static final double SPOOL_GEAR_RATIO = 24.0 / 16.0; // Driver / Follower
    private static final double TICKS_PER_ROTATION = 384.5;
    private static final double ERROR_FACTOR = 29.0 / 25.2;
    private static final double INCHES_PER_MOTOR_TICK = ERROR_FACTOR * (SPOOL_GEAR_RATIO * SPOOL_CIRCUMFERENCE_INCHES) / TICKS_PER_ROTATION;
    private static final double TICKS_PER_INCHES = 1 / INCHES_PER_MOTOR_TICK;

    private static final double PITCH_GEAR_RATIO = 40.0 / 48.0; // Driver / Follower
    private static final double ROTATION_GEAR_RATIO = 1.0 / 1.0; // Driver / Follower

    public static double FORWARD_KP = 0.15;
    public static double FORWARD_KI = 0;
    public static double FORWARD_KD = 0;
    public static double FORWARD_KF = 0.18;
    public static double REVERSE_KP = 0.01;
    public static double REVERSE_KI = 0;
    public static double REVERSE_KD = 0;
    public static double REVERSE_KF = 0;
    public static double HOLD_KF = 0.2;
    public static double SLIDES_TOLERANCE = 1;


    private final SlideController slideController;

    /**
     * Initializes dropper subsystem
     *
     * @param hardwareMap: is a variable where you configure all the devices in the specific subsystem
     */
    public DropperSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        FtcDashboard ftcDashboard = FtcDashboard.getInstance();

        this.robotState = robotState;
        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_dropper_slide");
        leftSlideMotor = hardwareMap.get(DcMotor.class, "left_dropper_slide");
        rightPitchServo = hardwareMap.get(Servo.class, "right_dropper_pitch");
        leftPitchServo = hardwareMap.get(Servo.class, "left_dropper_pitch");
        rotationServo = hardwareMap.get(Servo.class, "dropper_rotation");
        grabServo = hardwareMap.get(Servo.class, "dropper_claw");

        PIDFCoefficients forwardPIDF = new PIDFCoefficients(FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_KF);
        PIDFCoefficients reversePIDF = new PIDFCoefficients(REVERSE_KP, REVERSE_KI, REVERSE_KD, REVERSE_KF);
        slideController = new SlideController(TICKS_PER_INCHES, forwardPIDF, reversePIDF, HOLD_KF);

        leftPitchServo.setDirection(Servo.Direction.REVERSE);
        rightPitchServo.setDirection(Servo.Direction.FORWARD);

        leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        encoderMotor = rightSlideMotor; // Assuming you are using leftSlideMotor to use as the encoder motor

        slideController.setTolerance(SLIDES_TOLERANCE);

        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Resets encoder values of each of the slide motors
     */
    public void resetSlides() {
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPos = 0;
    }
// TODO: Fix all the comments which say input is in degrees (it is in servo position)
    /**
     * Method that moves servo to make the claw open
     */
    public void openClaw() {
        grabServo.setPosition(0);
    }

    /**
     * Method that moves servo to make the claw close
     */
    public void closeClaw() {
        grabServo.setPosition(1);
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
    public double getCurrentPositionInInches() {
        return encoderMotor.getCurrentPosition() * INCHES_PER_MOTOR_TICK;
    }

    public double getTargetPositionInches() {
        return targetPos;
    }


    /**
     * Increments slides from wherever it is currently
     *
     * @param position: Amount you are incrementing by inches
     */
    public void moveSlidesRelative(double position) {
        slideController.moveToInches(getCurrentPositionInInches() + position);
        targetPos += position;
    }

    /**
     * Moves slides to that position from wherever it is
     *
     * @param position: Position where you want to set the slides to in inches
     */
    public void moveSlidesAbsoluteInches(double position) {
        slideController.moveToInches(position);
        targetPos = position;
    }

    /**
     * Sets a position and rotation for the dropper arm
     *
     * @param pitch: Angle to set the dropper arm in degrees
     * @param rotation: Angle to set the dropper claw in degrees
     */
    public void setWristAbsolute(double pitch, double rotation) {
        rightPitchServo.setPosition(rightPitchServo.getPosition() + PITCH_GEAR_RATIO * pitch);
        leftPitchServo.setPosition(leftPitchServo.getPosition() + PITCH_GEAR_RATIO * pitch);
        rotationServo.setPosition(rotationServo.getPosition() + ROTATION_GEAR_RATIO * rotation);
    }

    /**
     * Adjusts the wrist's position and angle from where it currently is
     *
     * @param pitch: Amount you want to increment the pitch in degrees
     * @param rotation: Amount you want to increment the wrist in degrees
     * */
    public void setWristRelative(double pitch, double rotation) {
        rightPitchServo.setPosition(rightPitchServo.getPosition() + PITCH_GEAR_RATIO * pitch);
        leftPitchServo.setPosition(leftPitchServo.getPosition() + PITCH_GEAR_RATIO * pitch);
        rotationServo.setPosition(rotationServo.getPosition() + ROTATION_GEAR_RATIO * rotation);
    }

    /**
     * Sets angle of arm to an absolute position
     *
     * @param pitch: Angle of the dropper arm in degrees
     * */
    // TODO: Fix the degrees thing - right now only using servo position
    public void setPitchAbsolute(double pitch) {
//        rightPitchServo.setPosition(pitch * PITCH_GEAR_RATIO);
//        leftPitchServo.setPosition(pitch * PITCH_GEAR_RATIO);
        rightPitchServo.setPosition(pitch);
        leftPitchServo.setPosition(pitch);
    }

    /**
     * Increments the pitch of the arm from where it currently is
     *
     * @param pitch: Amount you want to increment the pitch in degrees
     * */
    public void setPitchRelative(double pitch) {
//        rightPitchServo.setPosition(rightPitchServo.getPosition() + PITCH_GEAR_RATIO * pitch);
//        leftPitchServo.setPosition(leftPitchServo.getPosition() + PITCH_GEAR_RATIO * pitch);
        rightPitchServo.setPosition(rightPitchServo.getPosition() + pitch);
        leftPitchServo.setPosition(leftPitchServo.getPosition() + pitch);
    }

    /**
     * Rotates claw to an absolute position
     *
     * @param rotation: Amount you want to set claw rotation to
     * */
    public void setRotationAbsolute(double rotation) {
        rotationServo.setPosition(rotation);
    }

    /**
     * Rotates the claw so it is facing down
     */
    public void rotateClawDown(){
        rotationServo.setPosition(0.05);
    }

    /**
     * Rotates the claw so it is facing up
     */
    public void rotateClawUp(){
        rotationServo.setPosition(0.6);
    }

    /**
     * Increments the rotation of the claw from where it currently is
     *
     * @param rotation: Amount you want to increment the rotation in degrees
     *
     */
    public void setRotationRelative(double rotation) {
//        rotationServo.setPosition(rotationServo.getPosition() + ROTATION_GEAR_RATIO * rotation);
        rotationServo.setPosition(rotationServo.getPosition() + rotation);

    }

    public double getPitchPos(){
        return rightPitchServo.getPosition();
    }

    public double getRotationPos(){
        return rotationServo.getPosition();
    }

    public double getClawPos(){
        return grabServo.getPosition();
    }

    /**
     * Sets given power to both the slide motors
     *
     * @param power: Amount of power you want to set the slide motors
     * */
    public void manualControlSlides(double power) {
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }

    @Override
    public void periodic() {
//        double currentPos = getCurrentPositionInInches();
//
//        if (Math.abs(targetPos - currentPos) < SLIDES_TOLERANCE) {
//            leftSlideMotor.setPower(HOLD_KF);
//            rightSlideMotor.setPower(HOLD_KF);
//            RobotLog.dd("tt-ss", "Holding Power: [%s]", String.valueOf(HOLD_KF));
//        } else {
//            double power = slideController.calculateMotorPowers(encoderMotor.getCurrentPosition());
//            leftSlideMotor.setPower(power);
//            rightSlideMotor.setPower(power);
//        }
    }
}
