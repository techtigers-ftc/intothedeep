package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

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
    private static final double ERROR_FACTOR = 1.0;
    private static final double DIST_PER_MOTOR_TICK = (SPOOL_GEAR_RATIO * SPOOL_CIRCUMFERENCE_INCHES) / TICKS_PER_ROTATION;
    private static final double MOTOR_TICKS_PER_INCH = (1.0 / DIST_PER_MOTOR_TICK) * ERROR_FACTOR;


    public static final double FORWARD_KP = 0.1;
    public static final double FORWARD_KI = 0.1;
    public static final double FORWARD_KD = 0.1;
    public static final double FORWARD_KF = 0.1;
    public static final double REVERSE_KP = 0.1;
    public static final double REVERSE_KI = 0.1;
    public static final double REVERSE_KD = 0.1;
    public static final double REVERSE_KF = 0.1;


    private final RobotState robotState;
    private final DcMotor leftSlideMotor;
    private final DcMotor rightSlideMotor;
    private final DcMotor encoderMotor;
    private final Servo leftWrist;
    private final Servo rightWrist;
    private final Servo leftClaw;
    private final Servo rightClaw;
    private SlideController slideController;


    /**
     * Initializes a new IntakeSubsystem
     *
     * @param hardwareMap the reference to the hardware components of the robot
     * @param robotState a reference to the state used to store information about the robot
     */
    public IntakeSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        super();
        this.robotState = robotState;
        leftSlideMotor = hardwareMap.get(DcMotor.class, "left_intake_slide_motor");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_intake_slide_motor");
        leftWrist = hardwareMap.get(Servo.class, "left_intake_wrist");
        rightWrist = hardwareMap.get(Servo.class, "right_intake_wrist");
        leftClaw = hardwareMap.get(Servo.class, "left_intake_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_intake_claw");

        slideController = new SlideController(MOTOR_TICKS_PER_INCH,
                new PIDFCoefficients(FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_KF),
                new PIDFCoefficients(REVERSE_KP, REVERSE_KI, REVERSE_KD, REVERSE_KF)
        );

        //Assuming that the encoder is connected to the leftSlideMotor
        encoderMotor = leftSlideMotor;

        //Configure Motors
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setDirection(DcMotor.Direction.REVERSE); //TODO: Check if this is the correct motor directions
        rightSlideMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Private method that Returns Current Position in inches
     */
    private double getCurrentPositionInches() {
        return encoderMotor.getCurrentPosition() * DIST_PER_MOTOR_TICK;
    }

    /**
     * Moves the Slides in Absolute units
     * @param distance The distance you want to move in absolute units
     */
    public void moveSlidesAbsolute(double distance){
        slideController.moveTo(distance);
    }

    /**
     * Moves slides in inches
     * @param distance The distance you want to move in inches
     */
    public void moveSlidesRelative(double distance){
        slideController.moveTo(getCurrentPositionInches() + distance);
    }

    /**
     * Moves Slides back to zero position
     */
    public void resetSlides(){
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Stops slides
     */
    public void stopSlides(){
        slideController.moveTo(getCurrentPositionInches());
        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
    }

    /**
     * Updates and powers motors every cycle
     */
    @Override
    public void periodic(){
        double power = slideController.calculateMotorPowers(encoderMotor.getCurrentPosition());
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }
}
