package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.SlideController;

import team.techtigers.base.CloseableSubsytem;

public class DropperSubsystem extends CloseableSubsytem {
    private DcMotor rightSlideMotor;
    private DcMotor leftSlideMotor;
    private Servo pitchServo1;
    private Servo pitchServo2;
    private Servo rotationServo;
    private Servo grabServo;
    private final RobotState robotState;

    private static final double SPOOL_CIRCUMFERENCE_INCHES = 1.27 * Math.PI;
    private static final double SPOOL_GEAR_RATIO = 24.0 / 16.0; // Driver / Follower
    private static final double TICKS_PER_ROTATION = 384.5;
    private static final double ERROR_FACTOR = 1.0;
    private static final double INCHES_PER_MOTOR_TICK = (SPOOL_GEAR_RATIO * SPOOL_CIRCUMFERENCE_INCHES) / TICKS_PER_ROTATION;
    private static final double TICKS_PER_INCHES = (1 / INCHES_PER_MOTOR_TICK) * ERROR_FACTOR;

    private static final double FORWARD_KP = 0.1;
    private static final double FORWARD_KI = 0.1;
    private static final double FORWARD_KD = 0.1;
    private static final double FORWARD_KF = 0.1;
    private static final double REVERSE_KP = 0.1;
    private static final double REVERSE_KI = 0.1;
    private static final double REVERSE_KD = 0.1;
    private static final double REVERSE_KF = 0.1;


    /**
     * @param hardwareMap: is a variable where you configure all the devices in the specific subsystem
     */

    public DropperSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        super();
        rightSlideMotor = hardwareMap.get(DcMotor.class, "placeholder_name");
        leftSlideMotor = hardwareMap.get(DcMotor.class, "placeholder_name");
        pitchServo1 = hardwareMap.get(Servo.class, "placeholder_name");
        pitchServo2 = hardwareMap.get(Servo.class, "placeholder_name");
        rotationServo = hardwareMap.get(Servo.class, "placeholder_name");
        grabServo = hardwareMap.get(Servo.class, "placeholder_name");
        this.robotState = robotState;
        PIDFCoefficients forwardPIDF = new PIDFCoefficients(FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_KF);
        PIDFCoefficients reversePIDF = new PIDFCoefficients(REVERSE_KP, REVERSE_KI, REVERSE_KD, REVERSE_KF);
        new SlideController(TICKS_PER_INCHES, forwardPIDF, reversePIDF);
    }

    /**
     *  Resets encoder values of each of the slide motors
     */

    public void resetSlides() {
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     Method that moves servo to make the claw open
     */
    public void openClaw() {
        grabServo.setPosition(0);
    }

    /**
     Method that moves servo to make the claw close
     */
    public void closeClaw() {
        grabServo.setPosition(1);
    }

    /**
    Stops the slides wherever it's currently at
     */
    public void stopSlides() {
        rightSlideMotor.setPower(0);
        leftSlideMotor.setPower(0);
    }

    /**
     Increments slides from wherever it is currently
     @param position: Amount you are incrementing by in encoder ticks*/
    public void moveSlidesRelative(double position) {
        rightSlideMotor.setTargetPosition((int) (rightSlideMotor.getCurrentPosition() + position));
    }

    /**
     Moves slides to that position for wherever it is
     @param position: Position where you want to set the slides to*/
    public void moveSlidesAbsolute(int position) {
        rightSlideMotor.setTargetPosition(position);
        leftSlideMotor.setTargetPosition(position);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Method that increments the wrist from where it is currently at
     * @param pitch: Amount you want to increment by in degrees
     */
    public void setWristRelative(double pitch, double rotation) {
        double pitchServoCurrentPosition  = pitchServo1.getPosition();
        double pitchServo2CurrentPosition = pitchServo2.getPosition();
        double rotationServoCurrentPosition = rotationServo.getPosition();

        pitchServo1.setPosition(pitchServoCurrentPosition + pitch);
        pitchServo2.setPosition(pitchServo2CurrentPosition + pitch);

        rotationServo.setPosition(rotationServoCurrentPosition + rotation);
    }

//    public void setWristRelativeRotation(double rotation) {
//        double rotationServoCurrentPosition = rotationServo.getPosition();
//        rotationServo.setPosition(rotationServoCurrentPosition + rotation);
//    }

    public void periodic() {

    }
}
