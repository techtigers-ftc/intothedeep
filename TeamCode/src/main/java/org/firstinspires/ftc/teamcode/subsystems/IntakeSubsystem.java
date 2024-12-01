package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.DifferentialController;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.SlideController;

import team.techtigers.base.CloseableSubsytem;

/**
 * A subsystem that controls all the motors for the intake subsystem.
 * Gives methods to control all of the aspects of the subsystem.
 * Controls both differential servos for the wrist, the two servos that control the claw, and the
 * two motors that control the horizontal slides.
 */
public class IntakeSubsystem extends CloseableSubsytem {
    private static final double SPOOL_CIRCUMFERENCE_INCHES = 1.27 * Math.PI;
    private static final double SPOOL_GEAR_RATIO = 1.0 / 1.0; // Driver / Follower
    private static final double TICKS_PER_ROTATION = 145.1;
    private static final double ERROR_FACTOR = 1.0 / 1.1565;
    private static final double DIST_PER_MOTOR_TICK = (SPOOL_GEAR_RATIO * SPOOL_CIRCUMFERENCE_INCHES) / TICKS_PER_ROTATION;
    private static final double MOTOR_TICKS_PER_INCH = (1.0 / DIST_PER_MOTOR_TICK) * ERROR_FACTOR;
    private static final double SERVO_GEAR_RATIO = 64.0 / 48.0; // Driver / Follower

    private static final double DIFFERENTIAL_GEAR_RATIO = 1.0 / 1.0; //Driver / Follower


    public static final double FORWARD_KP = 0.025;
    public static final double FORWARD_KI = 0.0;
    public static final double FORWARD_KD = 0.0;
    public static final double FORWARD_KF = 0.0;
    public static final double REVERSE_KP = 0.0;
    public static final double REVERSE_KI = 0.0;
    public static final double REVERSE_KD = 0.0;
    public static final double REVERSE_KF = 0.0;


    private final RobotState robotState;
    private final DcMotor leftSlideMotor;
    private final DcMotor rightSlideMotor;
    private final DcMotor encoderMotor;
    private final Servo leftWrist;
    private final Servo rightWrist;
    private final Servo leftClaw;
    private final Servo rightClaw;
    private final SlideController slideController;
    private final DifferentialController differentialController;


    /**
     * Initializes a new IntakeSubsystem
     *
     * @param hardwareMap the reference to the hardware components of the robot
     * @param robotState  a reference to the state used to store information about the robot
     */
    public IntakeSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        super();
        this.robotState = robotState;
        leftSlideMotor = hardwareMap.get(DcMotor.class, "left_intake_slide");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_intake_slide");
        leftWrist = hardwareMap.get(Servo.class, "left_intake_wrist");
        rightWrist = hardwareMap.get(Servo.class, "right_intake_wrist");
        leftClaw = hardwareMap.get(Servo.class, "left_intake_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_intake_claw");

        slideController = new SlideController(MOTOR_TICKS_PER_INCH,
                new PIDFCoefficients(FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_KF),
                new PIDFCoefficients(REVERSE_KP, REVERSE_KI, REVERSE_KD, REVERSE_KF)
        );
        differentialController = new DifferentialController(DIFFERENTIAL_GEAR_RATIO, 270, SERVO_GEAR_RATIO);//TODO: Find max servo angle
        differentialController.setMaxRange(180, 180);

        //Assuming that the encoder is connected to the leftSlideMotor
        encoderMotor = leftSlideMotor;

        //Configure Motors
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        rightSlideMotor.setDirection(DcMotor.Direction.REVERSE);

        rightWrist.setDirection(Servo.Direction.FORWARD);
        leftWrist.setDirection(Servo.Direction.REVERSE);

        rightClaw.setDirection(Servo.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.FORWARD);

        rightClaw.setPosition(0);
        leftClaw.setPosition(0);

        rightWrist.setPosition(0.5);
        leftWrist.setPosition(0.5);

        RobotLog.dd("IntakeSubsystem", "TicksPerInch: %f", MOTOR_TICKS_PER_INCH);
    }

    /**
     * Private method that Returns Current Position in inches
     */
    public double getCurrentPositionInches() {
        return encoderMotor.getCurrentPosition() / MOTOR_TICKS_PER_INCH;
    }

    /**
     * Moves the Slides to an exact position
     *
     * @param distance The distance you want to move in inches
     */
    public void moveSlidesAbsolute(double distance) {
        slideController.moveTo(distance);
    }

    /**
     * Moves slides in to a position relative to where it already is
     *
     * @param distance The distance you want to move in inches
     */
    public void moveSlidesRelative(double distance) {
        slideController.moveTo(getCurrentPositionInches() + distance);
    }

    /**
     * Sets the current position to 0 encoder ticks on the motors
     */
    public void resetSlides() {
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Stops slides
     */
    public void stopSlides() {
        slideController.moveTo(getCurrentPositionInches());
        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
    }

    /**
     * Opens The Intake Claw
     */
    public void openClaw() {
        leftClaw.setPosition(0);
        rightClaw.setPosition(0);
    }

    /**
     * Closes the Intake Claw
     */
    public void closeClaw() {
        leftClaw.setPosition(1);
        rightClaw.setPosition(1);
    }

    /**
     * Sets wrist position in degrees
     *
     * @param pitchAngle    the desired pitch of the differential
     * @param rotationAngle the desired rotation of the differential claw
     */
    public void setWristAbsolute(double pitchAngle, double rotationAngle) {
        double[] positions = differentialController.calculateServoPositions(pitchAngle, rotationAngle);
        leftWrist.setPosition(positions[0]);
        rightWrist.setPosition(positions[1]);
    }

    /**
     * Changes wrist position relative to where it is in degrees
     *
     * @param pitchAngle    the desired change in pitch of the differential
     * @param rotationAngle the desired change in rotation of the differential claw
     */
    public void setWristRelative(double pitchAngle, double rotationAngle) {
        double[] currentPositions = differentialController.getPitchAndRotation(
                leftWrist.getPosition(),
                rightWrist.getPosition());
        double[] newPositions = differentialController.calculateServoPositions(
                currentPositions[0] + pitchAngle,
                currentPositions[1] + rotationAngle);
        leftWrist.setPosition(newPositions[0]);
        rightWrist.setPosition(newPositions[1]);
    }

    public void powerSlides(double power) {
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }

    /**
     * Updates and powers motors every cycle
     */
    @Override
    public void periodic() {
//        double power = slideController.calculateMotorPowers(encoderMotor.getCurrentPosition());
//        leftSlideMotor.setPower(power);
//        rightSlideMotor.setPower(power);
        double[] wristAngles = differentialController.getPitchAndRotation(leftWrist.getPosition(),
                rightWrist.getPosition());
        double[] wristPositions = differentialController.calculateServoPositions(wristAngles[0], wristAngles[1]);
        RobotLog.dd("IntakeSubsystem", "Wrist Pitch: %f Wrist Rotation: %f", wristAngles[0], wristAngles[1]);
        RobotLog.dd("IntakeSubsystem", "Actual Left Wrist: %f Actual Right Wrist: %f", leftWrist.getPosition(),
                rightWrist.getPosition());
        RobotLog.dd("IntakeSubsystem", "Calculated Left Wrist: %f Calculated Right Wrist: %f", wristPositions[0],
                wristPositions[1]);
    }
}
