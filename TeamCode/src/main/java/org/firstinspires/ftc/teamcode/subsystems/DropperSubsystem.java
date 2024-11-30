package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

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
    public static double SLIDES_TOLERANCE = 0.05;

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
        slideController = new SlideController(TICKS_PER_INCHES, forwardPIDF, reversePIDF);

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
     * Method that increments the wrist from where it is currently at
     *
     * @param pitch: Amount you want to increment by for the dropper in degrees
     */
    public void setWristRelativeDegrees(double pitch, double rotation) {
        double pitchServoCurrentPosition = rightPitchServo.getPosition();
        double pitchServo2CurrentPosition = leftPitchServo.getPosition();
        double rotationServoCurrentPosition = rotationServo.getPosition();

        rightPitchServo.setPosition(pitchServoCurrentPosition * PITCH_GEAR_RATIO + pitch);
        leftPitchServo.setPosition(pitchServo2CurrentPosition * PITCH_GEAR_RATIO + pitch);

        rotationServo.setPosition(rotationServoCurrentPosition * ROTATION_GEAR_RATIO + rotation);
    }

    /**
     * Sets the wrist to an absolute position in degrees
     *
     * @param pitch    The angle to set the pitch to in degrees
     * @param rotation The angle to set the rotation to in degrees
     */
    public void setWristAbsoluteDegrees(double pitch, double rotation) {
        rightPitchServo.setPosition(pitch * PITCH_GEAR_RATIO);
        leftPitchServo.setPosition(pitch * PITCH_GEAR_RATIO);
        rotationServo.setPosition(rotation * ROTATION_GEAR_RATIO);
    }

    public void manualControlSlides(double power) {
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }

    @Override
    public void periodic() {
        double power = slideController.calculateMotorPowers(encoderMotor.getCurrentPosition());
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }
}
