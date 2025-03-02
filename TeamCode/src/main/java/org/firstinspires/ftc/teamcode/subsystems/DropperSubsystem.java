package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.DifferentialController;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.SlideController;
import org.firstinspires.ftc.teamcode.utils.SlidingAverageCalculator;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.CloseableSubsystem;

/**
 * Encapsulates all hardware, methods, and attributes of the dropper subsystem, including the
 * vertical slides, arm, and the claw.
 */
@Config
public class DropperSubsystem extends CloseableSubsystem {
    // SLIDE POSITIONS
    public static final double SLIDE_MAX = 28.25;
    public static final double SLIDES_PRE_TRANSFER_POSITION = 6;
    public static final double SLIDES_TRANSFER_POSITION = 0.75;
    public static final double SLIDES_CHAMBER_POSITION = 5;
    public static final double SLIDES_WALL_INTAKE_POSITION = 0;

    // PITCH POSITIONS
    public static final double PITCH_PRE_TRANSFER_POSITION = 90;
    public static final double PITCH_TRANSFER_POSITION = 40;
    public static final double PITCH_BASKET_POSITION = 230;
    public static final double PITCH_CHAMBER_POSITION = 155; // 180
    public static final double PITCH_FRONT_SLAP_POSITION = 90;
    public static final double PITCH_BACK_SLAP_POSITION = 265;
    public static final double PITCH_WALL_INTAKE_POSITION = 310;

    // ROTATION POSITIONS
    public static final double ROTATION_TRANSFER_POSITION = 210;
    public static final double ROTATION_BASKET_POSITION = 210;
    public static final double ROTATION_FRONT_SLAP_POSITION = 210;
    public static final double ROTATION_BACK_SLAP_POSITION = 10;
    public static final double ROTATION_WALL_INTAKE_POSITION = 10;

    private static final double SPOOL_CIRCUMFERENCE_INCHES = 1.758 * Math.PI;
    private static final double SPOOL_GEAR_RATIO = 1.0; // Driver / Follower
    private static final double TICKS_PER_ROTATION = 384.5;
    private static final double ERROR_FACTOR = 1;
    private static final double INCHES_PER_MOTOR_TICK = ERROR_FACTOR * (SPOOL_GEAR_RATIO * SPOOL_CIRCUMFERENCE_INCHES) / TICKS_PER_ROTATION;
    private static final double TICKS_PER_INCHES = 1 / INCHES_PER_MOTOR_TICK;
    private static final double GEAR_RATIO = 1;
    private static final double SERVO_GEAR_RATIO = 40.0 / 26.0;
    public static double CLAW_OPENED_POSITION = 0.6;
    public static double CLAW_CLOSED_POSITION = 0.24;
    public static double PRIMARY_KP = 0.006;
    public static double PRIMARY_KI = 0;
    public static double PRIMARY_KD = 0;
    public static double PRIMARY_KF = 0;
    public static double SECONDARY_KP = 0.006;
    public static double SECONDARY_KI = 0;
    public static double SECONDARY_KD = 0;
    public static double SECONDARY_KF = 0;
    public static double SLIDES_TOLERANCE = 1;
    public final DcMotor rightSlideMotor;
    public final DcMotor leftSlideMotor;
    private final PIDFCoefficients PRIMARY_COEFFICIENTS = new PIDFCoefficients(PRIMARY_KP, PRIMARY_KI, PRIMARY_KD, PRIMARY_KF);
    private final PIDFCoefficients SECONDARY_COEFFICIENTS = new PIDFCoefficients(SECONDARY_KP, SECONDARY_KI, SECONDARY_KD, SECONDARY_KF);
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
    private boolean inPrimarySlideMode;

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

        slideController = new SlideController(TICKS_PER_INCHES, PRIMARY_COEFFICIENTS);
        inPrimarySlideMode = true;
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

        slideController.setTolerance(SLIDES_TOLERANCE);

        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (robotState.isAuto()) {
            closeClaw();
            resetSlides();
            setWristAbsolute(PITCH_PRE_TRANSFER_POSITION, ROTATION_TRANSFER_POSITION);
        }
    }

    @Override
    public void init() {
        if (!robotState.isAuto()) {
            setWristAbsolute(PITCH_PRE_TRANSFER_POSITION, ROTATION_TRANSFER_POSITION);
            if (getCurrentSlidePositionInches() > 5) {
                moveSlidesAbsolute(getCurrentSlidePositionInches());
                closeClaw();
            } else {
                openClaw();
            }
        }
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
        return slideController.getTargetTicks() * INCHES_PER_MOTOR_TICK;
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
     * Increments slides from wherever it is currently, with no limits
     *
     * @param position Amount you are incrementing by inches
     */
    public void moveSlidesRelativeUnsafe(double position) {
        slideController.moveToInches(getCurrentSlidePositionInches() + position);
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
    public double getSlideCurrentRight() {
        return rightSlideCurrentAverage.getAverage();
    }

    /**
     * @return the current draw of the right slide motor
     */
    public double getSlideCurrentLeft() {
        return leftSlideCurrentAverage.getAverage();
    }

    private double getVoltageCompensatedMotorPower(double power) {
        if (robotState.getVoltage() != 0) {
            return Range.clip(power / (robotState.getVoltage() / 12.0), -1, 1);
        } else {
            return power;
        }
    }

    @Override
    public void periodic() {
        if (!robotState.getIsAscending()) {
            if (getCurrentSlidePositionInches() > 23 && inPrimarySlideMode) {
                slideController.setPIDFCoefficients(SECONDARY_COEFFICIENTS);
                inPrimarySlideMode = false;
            } else if (getCurrentSlidePositionInches() < 23 && !inPrimarySlideMode) {
                slideController.setPIDFCoefficients(PRIMARY_COEFFICIENTS);
                inPrimarySlideMode = true;
            }
            double power = getVoltageCompensatedMotorPower(slideController.calculateMotorPowers(getCurrentSlidePositionTicks()));
            leftSlideMotor.setPower(power);
            rightSlideMotor.setPower(power);
        }

        robotState.setVerticalExtended(getCurrentSlidePositionTicks() > 100);
        leftSlideCurrentAverage.add(currentMotorLeft.getCurrent(CurrentUnit.AMPS));
        rightSlideCurrentAverage.add(currentMotorRight.getCurrent(CurrentUnit.AMPS));

        robotState.setDropperCurrent(rightSlideCurrentAverage.getAverage() + leftSlideCurrentAverage.getAverage());

//        if(colorSensorTimer.milliseconds() > 1000){
//            updateBlockColor();
//            colorSensorTimer.reset();
//        }


    }
}
