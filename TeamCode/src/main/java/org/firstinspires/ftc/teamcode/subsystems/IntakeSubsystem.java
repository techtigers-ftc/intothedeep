package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.DifferentialController;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.SlideController;
import org.firstinspires.ftc.teamcode.utils.SlidingAverageCalculator;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColor;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem that controls all the motors for the intake subsystem.
 * Gives methods to control all of the aspects of the subsystem.
 * Controls both differential servos for the wrist, the two servos that control the claw, and the
 * two motors that control the horizontal slides.
 */
@Config
public class IntakeSubsystem extends CloseableSubsystem {
    public static double minMagnitude = 1;
    public static double minBlue = 0.53;
    public static double minRed = 0.43;
    public static double FORWARD_KP = 0.00475;
    public static double FORWARD_KI = 0.0;
    public static double FORWARD_KD = 0.0001;
    public static double FORWARD_KF = 0.06;
    private static final double SPOOL_CIRCUMFERENCE_INCHES = 1.27 * Math.PI;
    private static final double SPOOL_GEAR_RATIO = 1.0; // Driver / Follower
    private static final double TICKS_PER_ROTATION = 145.1;
    private static final double ERROR_FACTOR = 1.0 / 1.1565;
    private static final double DIST_PER_MOTOR_TICK = (SPOOL_GEAR_RATIO * SPOOL_CIRCUMFERENCE_INCHES) / TICKS_PER_ROTATION;
    private static final double MOTOR_TICKS_PER_INCH = (1.0 / DIST_PER_MOTOR_TICK) * ERROR_FACTOR;
    private static final double SERVO_GEAR_RATIO = 64.0 / 48.0; // Driver / Follower
    private static final double DIFFERENTIAL_GEAR_RATIO = 0.9; //Driver / Follower
    private static final double CLAW_OPEN_POSITION = 0.25;
    private static final double CLAW_MIDDLE_POSITION = 0.55;
    private static final double CLAW_LOOSE_POSITION = 0.75;
    private static final double CLAW_CLOSED_POSITION = 0.8;
    private static final double INTAKE_CLAW_ROTATION_RANGE = 180;
    public static final double SLIDES_MAX = 19;

    public static final double WRIST_PITCH_TUCK_POSITION = 0;
    public static final double WRIST_ROTATION_TUCK_POSITION = 0;
    public static final double CLAW_ROTATION_TUCK_POSITION = 90;

    public static final double WRIST_PITCH_PREPARE_TO_PICKUP_POSITION = 0;
    public static final double WRIST_ROTATION_PREPARE_TO_PICKUP_POSITION = 0;
    public static final double CLAW_ROTATION_PICKUP_POSITION = 90;

    public static final double WRIST_PITCH_READY_TO_PICKUP_POSITION = 70;
    public static final double WRIST_ROTATION_READY_TO_PICKUP_POSITION = 170;

    public static final double WRIST_PITCH_PECK_POSITION = 90;

    public static final double WRIST_PITCH_TRANSFER_POSITION = 45;
    public static final double WRIST_ROTATION_TRANSFER_POSITION = 0;
    public static final double CLAW_ROTATION_TRANSFER_POSITION = 90;


    private final RobotState robotState;
    private final DcMotor leftSlideMotor;
    private final DcMotor rightSlideMotor;
    private final DcMotor encoderMotor;
    private final DcMotorEx currentMotorRight;
    private final DcMotorEx currentMotorLeft;
    private final Servo leftWrist;
    private final Servo rightWrist;
    private final Servo leftClaw;
    private final Servo rightClaw;
    private final Servo clawRotation;
    private final SlideController slideController;
    private final DifferentialController differentialController;
    private final SlidingAverageCalculator leftSlideCurrentAverage;
    private final SlidingAverageCalculator rightSlideCurrentAverage;
    private final NormalizedColorSensor colorSensor;
    private final ElapsedTime colorSensorTimer;

    /**
     * Initializes a new IntakeSubsystem
     *
     * @param hardwareMap the reference to the hardware components of the robot
     * @param robotState  a reference to the state used to store information about the robot
     */
    public IntakeSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        leftSlideMotor = hardwareMap.get(DcMotor.class, "left_intake_slide");
        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_intake_slide");
        //Zero for wrist is straight forward, with the claw facing downwards
        leftWrist = hardwareMap.get(Servo.class, "left_intake_wrist");
        rightWrist = hardwareMap.get(Servo.class, "right_intake_wrist");
        //Claw zero is the most open position of the claw
        leftClaw = hardwareMap.get(Servo.class, "left_intake_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_intake_claw");
        //Claw rotation zero is perpendicular to the slides, the triangle facing forwards
        clawRotation = hardwareMap.get(Servo.class, "intake_claw_rotation");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intake_color_sensor");

        slideController = new SlideController(MOTOR_TICKS_PER_INCH, new PIDFCoefficients(FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_KF));
        differentialController = new DifferentialController(DIFFERENTIAL_GEAR_RATIO, 270, SERVO_GEAR_RATIO);//TODO: Find max servo angle
        differentialController.setMaxRange(180, 180);
        rightSlideCurrentAverage = new SlidingAverageCalculator(10);
        leftSlideCurrentAverage = new SlidingAverageCalculator(10);

        //Assuming that the encoder is connected to the leftSlideMotor
        encoderMotor = rightSlideMotor;
        resetSlides();
        currentMotorRight = (DcMotorEx) rightSlideMotor;
        currentMotorLeft = (DcMotorEx) leftSlideMotor;

        //Configure Motors
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        rightSlideMotor.setDirection(DcMotor.Direction.REVERSE);

        rightWrist.setDirection(Servo.Direction.FORWARD);
        leftWrist.setDirection(Servo.Direction.REVERSE);

        rightClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.setDirection(Servo.Direction.REVERSE);
        colorSensorTimer = new ElapsedTime();

        if (robotState.isAuto()) {
            init();
        }

        RobotLog.dd("IntakeSubsystem", "TicksPerInch: %f", MOTOR_TICKS_PER_INCH);
    }

    @Override
    public void init() {
        // Pitch init is in the transfer position
        //Rotation init is in the transfer position
        // Rotation zero is pointing parallel to the robot
        setWristAbsolute(WRIST_PITCH_TUCK_POSITION, WRIST_ROTATION_TUCK_POSITION);
        setClawRotationAbsolute(CLAW_ROTATION_TUCK_POSITION);
        closeClaw();
    }

    /**
     * @return current slide position in inches
     */
    public double getCurrentSlidePositionInches() {
        return encoderMotor.getCurrentPosition() / MOTOR_TICKS_PER_INCH;
    }

    /**
     * @return the pitch of the wrist in degrees
     */
    public double getPitch() {
        return differentialController.getPitchAndRotation(leftWrist.getPosition(), rightWrist.getPosition())[0];
    }

    /**
     * @return the rotation of the wrist in degrees
     */
    public double getRotation() {
        return differentialController.getPitchAndRotation(leftWrist.getPosition(), rightWrist.getPosition())[1];
    }

    /**
     * @return the position of the claw
     */
    public double getClawPosition() {
        return leftClaw.getPosition();
    }

    /**
     * @return the rotation of the claw in degrees
     */
    public double getClawRotation() {
        return clawRotation.getPosition() * INTAKE_CLAW_ROTATION_RANGE;
    }

    /**
     * Moves the Slides to an exact position
     *
     * @param distance The distance you want to move in inches
     */
    public void moveSlidesAbsolute(double distance) {
        slideController.moveToInches(Range.clip(distance, 0, SLIDES_MAX));
    }

    /**
     * Moves slides in to a position relative to where it already is
     *
     * @param distance The distance you want to move in inches
     */
    public void moveSlidesRelative(double distance) {
        moveSlidesAbsolute(getCurrentSlidePositionInches() + distance);
    }

    /**
     * Sets the current position to 0 encoder ticks on the motors
     */
    public void resetSlides() {
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        moveSlidesAbsolute(0);
    }

    /**
     * @return the target position of the slides in inches
     */
    public double getTargetPositionInches() {
        return slideController.targetTicks * MOTOR_TICKS_PER_INCH;
    }

    /**
     * Stops slides
     */
    public void stopSlides() {
        slideController.moveToInches(getCurrentSlidePositionInches());
        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
    }

    /**
     * Opens The Intake Claw
     */
    public void openClaw() {
        if (getPitch() <= IntakeSubsystem.WRIST_PITCH_READY_TO_PICKUP_POSITION - 5) {
            leftClaw.setPosition(CLAW_MIDDLE_POSITION);
            rightClaw.setPosition(CLAW_MIDDLE_POSITION);
        } else {
            leftClaw.setPosition(CLAW_OPEN_POSITION);
            rightClaw.setPosition(CLAW_OPEN_POSITION);
        }
        robotState.setIntakeClawState(ClawState.OPEN);
    }

    /**
     * Closes the Intake Claw
     */
    public void closeClaw() {
        leftClaw.setPosition(CLAW_CLOSED_POSITION);
        rightClaw.setPosition(CLAW_CLOSED_POSITION);
        robotState.setIntakeClawState(ClawState.CLOSED);
    }

    /**
     * Loosens the intake claw
     */
    public void loosenClaw() {
        leftClaw.setPosition(CLAW_LOOSE_POSITION);
        rightClaw.setPosition(CLAW_LOOSE_POSITION);
        robotState.setIntakeClawState(ClawState.CLOSED);
    }

    /**
     * Toggles the claw between open and closed
     */
    public void toggleClaw() {
        if (robotState.getIntakeClawState() == ClawState.CLOSED) {
            openClaw();
        } else {
            closeClaw();
        }
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
        robotState.setIntakeClawPitch(pitchAngle);
        robotState.setIntakeClawRotation(rotationAngle);
    }

    /**
     * Changes wrist position relative to where it is in degrees
     *
     * @param pitchAngle    the desired change in pitch of the differential
     * @param rotationAngle the desired change in rotation of the differential claw
     */
    public void setWristRelative(double pitchAngle, double rotationAngle) {
        setWristAbsolute(getPitch() + pitchAngle, getRotation() + rotationAngle);
    }

    /**
     * Sets the pitch of the wrist, while keeping the rotation the same
     *
     * @param pitchAngle the desired pitch of the wrist
     */
    public void setWristPitchAbsolute(double pitchAngle) {
        setWristAbsolute(pitchAngle, getRotation());
    }

    /**
     * Sets the pitch of the wrist relative to where it is, while keeping the rotation the same
     *
     * @param pitchAngle the desired pitch of the wrist
     */
    public void setWristPitchRelative(double pitchAngle) {
        setWristRelative(pitchAngle, 0);
    }


    /**
     * Sets the rotation of the wrist relative to its current position,
     * while keeping the pitch the same
     *
     * @param rotationAngle the desired change in rotation of the wrist
     */
    public void setWristRotationRelative(double rotationAngle) {
        setWristRelative(0, rotationAngle);
    }

    /**
     * Sets the rotation of the wrist, while keeping the pitch the same
     *
     * @param rotationAngle the desired rotation of the wrist
     */
    public void setWristRotationAbsolute(double rotationAngle) {
        setWristAbsolute(getPitch(), rotationAngle);
    }

    /**
     * Toggles the rotation of the wrist between 0 and 90
     */
    public void togglePerpendicularRotation() {
        if (getClawRotation() == 90) {
            setClawRotationAbsolute(0);
        } else {
            setClawRotationAbsolute(90);
        }
    }

    /**
     * Sets the rotation of the claw.
     *
     * @param rotationAngle the desired rotation of the claw in degrees
     */
    public void setClawRotationAbsolute(double rotationAngle) {
        clawRotation.setPosition(rotationAngle / INTAKE_CLAW_ROTATION_RANGE);
    }

    /**
     * Sets the rotation of the claw relative to its current position.
     *
     * @param rotationAngle the desired change in rotation of the claw in degrees
     */
    public void setClawRotationRelative(double rotationAngle) {
        setClawRotationAbsolute(getClawRotation() + rotationAngle);
    }

    /**
     * @return the current draw of the right slide motor
     */
    public double getSlideCurrentRight() {
        return rightSlideCurrentAverage.getAverage();
    }

    /**
     * @return the current draw of the left slide motor
     */
    public double getSlideCurrentLeft(){
        return leftSlideCurrentAverage.getAverage();
    }

    /**
     * Updates the block color of the color sensor
     */
    private void updateBlockColor() {
        double sensorRed = getSensorRed();
        double sensorGreen = getSensorGreen();
        double sensorBlue = getSensorBlue();
        double colorsSum = sensorRed + sensorGreen + sensorBlue;
        double normalizedBlue = sensorBlue / colorsSum;
        double normalizedGreen = sensorGreen / colorsSum;
        double normalizedRed = sensorRed / colorsSum;
        int magnitude = (int) Math.sqrt(Math.pow(sensorBlue, 2) + Math.pow(sensorRed, 2) + Math.pow(sensorGreen, 2));

        RobotLog.dd(tag, "Normalized Colors Red: %f, Green: %f, Blue: %f", normalizedRed, normalizedGreen, normalizedBlue);
        RobotLog.dd(tag, "UnNormalized Colors Red: %f, Green: %f, Blue: %f", sensorRed, sensorGreen, sensorBlue);

        if (magnitude < minMagnitude) {
            robotState.setIntakeBlockColor(BlockColor.NONE);
        } else if (normalizedBlue > minBlue) {
            robotState.setIntakeBlockColor(BlockColor.BLUE);
        } else if (normalizedRed > minRed) {
            robotState.setIntakeBlockColor(BlockColor.RED);
        } else if (normalizedBlue < 0.165) {
            robotState.setIntakeBlockColor(BlockColor.YELLOW);
        }
    }

    /**
     * @return the blue value of the color sensor
     */
    public double getSensorBlue() {
        return (colorSensor.getNormalizedColors().toColor() & 0xFF);
    }

    /**
     * @return the red value of the color sensor
     */
    public double getSensorRed() {
        return (colorSensor.getNormalizedColors().toColor() >> 16 & 0xFF);
    }

    /**
     * @return the green value of the color sensor
     */
    public double getSensorGreen() {
        return (colorSensor.getNormalizedColors().toColor() >> 8 & 0xFF);
    }

    /**
     * Updates and powers motors every cycle
     */
    @Override
    public void periodic() {
        double power = slideController.calculateMotorPowers(encoderMotor.getCurrentPosition());
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);

        robotState.setHorizontalExtended(encoderMotor.getCurrentPosition() > 100);
        double[] wristAngles = differentialController.getPitchAndRotation(leftWrist.getPosition(), rightWrist.getPosition());
        double[] wristPositions = differentialController.calculateServoPositions(wristAngles[0], wristAngles[1]);

        leftSlideCurrentAverage.add(currentMotorLeft.getCurrent(CurrentUnit.AMPS));
        rightSlideCurrentAverage.add(currentMotorRight.getCurrent(CurrentUnit.AMPS));

        robotState.setIntakeCurrent(rightSlideCurrentAverage.getAverage() + leftSlideCurrentAverage.getAverage());

//        if(colorSensorTimer.milliseconds() > 1000){
//            updateBlockColor();
//            colorSensorTimer.reset();
//        }

        RobotLog.dd(tag, "Wrist Pitch: %f Wrist Rotation: %f", wristAngles[0], wristAngles[1]);
        RobotLog.dd(tag, "Actual Left Wrist: %f Actual Right Wrist: %f", leftWrist.getPosition(), rightWrist.getPosition());
        RobotLog.dd(tag, "Calculated Left Wrist: %f Calculated Right Wrist: %f", wristPositions[0], wristPositions[1]);
        RobotLog.dd(tag, "Current Slide Position: %f", getCurrentSlidePositionInches());
        RobotLog.dd(tag, "Left Slide Current: %f", leftSlideCurrentAverage.getAverage());
        RobotLog.dd(tag, "Right Slide Current: %f", rightSlideCurrentAverage.getAverage());

        RobotLog.dd(tag, "Intake Block Color: %s", robotState.getIntakeBlockColor().toString());
    }
}
