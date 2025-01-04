package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.DifferentialController;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.SlideController;
import org.firstinspires.ftc.teamcode.utils.SlidingAverageCalculator;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColor;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.CloseableSubsystem;

/**
 * Encapsulates all hardware, methods, and attributes of the dropper subsystem, including the
 * vertical slides, arm, and the claw.
 */
@Config
public class DropperSubsystem extends CloseableSubsystem {
    public static final double PITCH_PRE_TRANSFER_POSITION = 37;
    public static final double PITCH_TRANSFER_POSITION = 20;
    public static final double PITCH_BASKET_POSITION = 200;
    public static final double PITCH_CHAMBER_POSITION = 175;
    public static final double PITCH_FRONT_SLAP_POSITION = 105;
    public static final double PITCH_BACK_SLAP_POSITION = 245;
    public static final double ROTATION_TRANSFER_POSITION = 10;
    public static final double ROTATION_BASKET_POSITION = 10;
    public static final double ROTATION_FRONT_SLAP_POSITION = 10;
    public static final double ROTATION_BACK_SLAP_POSITION = 210;
    private static final double SPOOL_CIRCUMFERENCE_INCHES = 1.27 * Math.PI;
    private static final double SPOOL_GEAR_RATIO = 1.0; // Driver / Follower
    private static final double TICKS_PER_ROTATION = 384.5;
    private static final double ERROR_FACTOR = 29.0 / 25.2 * 0.97;
    private static final double INCHES_PER_MOTOR_TICK = ERROR_FACTOR * (SPOOL_GEAR_RATIO * SPOOL_CIRCUMFERENCE_INCHES) / TICKS_PER_ROTATION;
    private static final double TICKS_PER_INCHES = 1 / INCHES_PER_MOTOR_TICK;
    private static final double SLIDE_MAX = 22;
    private static final double GEAR_RATIO = 1;
    private static final double SERVO_GEAR_RATIO = 40.0 / 26.0;
    public static double CLAW_OPENED_POSITION = 0.9;
    public static double CLAW_CLOSED_POSITION = 0.02;
    public static double KP = 0.015;
    public static double KI = 0;
    public static double KD = 0.000000001;
    public static double KF = 0;
    public static double SLIDES_TOLERANCE = 1;
    private final DcMotor rightSlideMotor;
    private final DcMotor leftSlideMotor;
    private final DcMotor encoderMotor;
    private final DcMotorEx currentMotorRight;
    private final DcMotorEx currentMotorLeft;
    private final Servo leftWrist;
    private final Servo rightWrist;
    private final Servo grabServo;
    private final RobotState robotState;
    private final SlideController slideController;
    private final DifferentialController differentialController;
    private final SlidingAverageCalculator leftSlideCurrentAverage;
    private final SlidingAverageCalculator rightSlideCurrentAverage;
    private final NormalizedColorSensor colorSensor;

    /**
     * Initializes dropper subsystem
     *
     * @param hardwareMap: is a variable where you configure all the devices in the specific subsystem
     */
    public DropperSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_dropper_slide");
        leftSlideMotor = hardwareMap.get(DcMotor.class, "left_dropper_slide");
        //Wrist zero is over against the bar, with the rotation in the transfer position
        leftWrist = hardwareMap.get(Servo.class, "left_dropper_wrist");
        rightWrist = hardwareMap.get(Servo.class, "right_dropper_wrist");
        //Claw zero is open
        grabServo = hardwareMap.get(Servo.class, "dropper_claw");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "dropper_color_sensor");

        PIDFCoefficients forwardPIDF = new PIDFCoefficients(KP, KI, KD, KF);
        slideController = new SlideController(TICKS_PER_INCHES, forwardPIDF);
        differentialController = new DifferentialController(GEAR_RATIO, 355, SERVO_GEAR_RATIO);
        differentialController.setMaxRange(330, 215);
        rightSlideCurrentAverage = new SlidingAverageCalculator(10);
        leftSlideCurrentAverage = new SlidingAverageCalculator(10);

        leftWrist.setDirection(Servo.Direction.REVERSE);
        rightWrist.setDirection(Servo.Direction.FORWARD);

        leftSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        encoderMotor = rightSlideMotor; // Assuming rightSlideMotor is the encoder motor
        currentMotorRight = (DcMotorEx) rightSlideMotor;
        currentMotorLeft = (DcMotorEx) leftSlideMotor;
        resetSlides();

        slideController.setTolerance(SLIDES_TOLERANCE);

        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setWristAbsolute(PITCH_PRE_TRANSFER_POSITION, ROTATION_TRANSFER_POSITION);
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
        if (robotState.getBlockPosition() == RobotBlockPosition.DROPPER) {
            robotState.setBlockPosition(RobotBlockPosition.NONE);
        }
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
        if (robotState.getDropperClawState() == ClawState.CLOSED) {
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
     * Gets the current position of the slides in ticks
     *
     * @return the current position of the slides in ticks
     */
    public double getCurrentSlidePositionTicks() {
        return -encoderMotor.getCurrentPosition();
    }

    /**
     * Gets the current position of the slides in inches
     *
     * @return the current position of the slides in inches
     */
    public double getCurrentSlidePositionInches() {
        return getCurrentSlidePositionTicks() * INCHES_PER_MOTOR_TICK;
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
     * Moves slides to that position from wherever it is
     *
     * @param position Position where you want to set the slides to in inches
     */
    public void moveSlidesAbsolute(double position) {
        slideController.moveToInches(Range.clip(position, 0, SLIDE_MAX));
    }

    /**
     * Increments slides from wherever it is currently
     *
     * @param position Amount you are incrementing by inches
     */
    public void moveSlidesRelative(double position) {
        moveSlidesAbsolute(getCurrentSlidePositionInches() + position);
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
        double[] positions = differentialController.calculateServoPositions(pitch, rotation);
        leftWrist.setPosition(positions[0]);
        rightWrist.setPosition(positions[1]);
        robotState.setDropperClawPitch(getPitch());
        robotState.setDropperClawRotation(getRotation());
    }

    /**
     * @return the pitch of the dropper arm in degrees
     */
    public double getPitch() {
        return differentialController.getPitchAndRotation(leftWrist.getPosition(), rightWrist.getPosition())[0];
    }

    /**
     * @return the rotation of the claw in degrees
     */
    public double getRotation() {
        return differentialController.getPitchAndRotation(leftWrist.getPosition(), rightWrist.getPosition())[1];
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
     * Sets the rotation of the wrist, while keeping the pitch the same
     *
     * @param rotationAngle the desired change in pitch of the wrist
     */
    public void setRotationRelative(double rotationAngle) {
        setWristRelative(0, rotationAngle);
    }

    /**
     * @return the current draw of the right slide motor
     */
    public double getSlideCurrentRight(){
        return rightSlideCurrentAverage.getAverage();
    }

    /**
     * @return the current draw of the right slide motor
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

        if (magnitude < 30) {
            robotState.setIntakeBlockColor(BlockColor.NONE);
        } else if (normalizedBlue > 0.53) {
            robotState.setIntakeBlockColor(BlockColor.BLUE);
        } else if (normalizedRed > 0.43) {
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

    @Override
    public void periodic() {
        double power =
                slideController.calculateMotorPowers(getCurrentSlidePositionTicks());
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);

        robotState.setVerticalExtended(getCurrentSlidePositionTicks() > 100);
        leftSlideCurrentAverage.add(currentMotorLeft.getCurrent(CurrentUnit.AMPS));
        rightSlideCurrentAverage.add(currentMotorRight.getCurrent(CurrentUnit.AMPS));

        robotState.setDropperCurrent(rightSlideCurrentAverage.getAverage() + leftSlideCurrentAverage.getAverage());

        // TODO: Add debounce
//        updateBlockColor();


        RobotLog.dd(tag, "Current: %f Target %f",
                getCurrentSlidePositionInches(), getTargetPositionInches());
        RobotLog.dd(tag, "Left Slide Current: %f", leftSlideCurrentAverage.getAverage());
        RobotLog.dd(tag, "Right Slide Current: %f", rightSlideCurrentAverage.getAverage());
        RobotLog.dd(tag, "Dropper Block Color: %s", robotState.getDropperBlockColor().toString());
    }
}
