package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem that controls all the motors for the intake subsystem.
 * Gives methods to control all of the aspects of the subsystem.
 * Controls both differential servos for the wrist, the two servos that control the claw, and the
 * two motors that control the horizontal slides.]
 */
@Config
public class  IntakeSubsystem extends CloseableSubsystem {
    public static double CLAW_ROTATION_BUFFER = 40;
    // Zero position: Wrist Pitch: 165, Wrist Rotation: 172, Claw Rotation: 90,
    // Claw closed
    // If zeroed correctly, going to a pitch of 50 should make the limelight perpendicular to the floor

    public static final double SLIDES_MAX = 18.75;
    public static final double WRIST_PITCH_TUCK_POSITION = 50;
    public static final double WRIST_ROTATION_TUCK_POSITION = 172;
    public static final double CLAW_ROTATION_TUCK_POSITION = 90;

    public static final double WRIST_PITCH_PREPARE_TO_PICKUP_POSITION = 91;
    public static final double WRIST_ROTATION_PREPARE_TO_PICKUP_POSITION = 172;

    public static final double CLAW_ROTATION_PICKUP_POSITION = 90;
    public static final double WRIST_PITCH_READY_TO_PICKUP_POSITION = 123;
    public static final double WRIST_ROTATION_READY_TO_PICKUP_POSITION = 172;

    public static final double WRIST_PITCH_PECK_POSITION = 159;

    public static final double WRIST_PITCH_TRANSFER_POSITION = 107;
    public static final double WRIST_ROTATION_TRANSFER_POSITION = 5;
    public static final double CLAW_ROTATION_TRANSFER_POSITION = 90;

    public static final double SLIDES_TRANSFER_POSITION = 0;

    private static final double SPOOL_CIRCUMFERENCE_INCHES = 1.26 * Math.PI;
    private static final double SPOOL_GEAR_RATIO = 1.0; // Driver / Follower
    private static final double TICKS_PER_ROTATION = 145.1;
    private static final double ERROR_FACTOR = 1.0 / 1.04247104;
    private static final double DIST_PER_MOTOR_TICK = (SPOOL_GEAR_RATIO * SPOOL_CIRCUMFERENCE_INCHES) / TICKS_PER_ROTATION;
    private static final double MOTOR_TICKS_PER_INCH = (1.0 / DIST_PER_MOTOR_TICK) * ERROR_FACTOR;
    private static final double SERVO_GEAR_RATIO = 64.0 / 48.0; // Driver / Follower
    private static final double DIFFERENTIAL_GEAR_RATIO = 0.9; //Driver / Follower
    public static double CLAW_OPEN_POSITION = 0.68;
    public static double CLAW_LOOSE_POSITION = 0.97;
    public static double CLAW_CLOSED_POSITION = 1;
    private static final double INTAKE_CLAW_ROTATION_RANGE = 270;
    public static double PRIMARY_KP = 0.007;
    public static double PRIMARY_KI = 0;
    public static double PRIMARY_KD = 0.0002;
    public static double PRIMARY_KF = 0.001;
    public static double SECONDARY_KP = 0.011;
    public static double SECONDARY_KI = 0;
    public static double SECONDARY_KD = 0;
    public static double SECONDARY_KF = 0;
    private final PIDFCoefficients PRIMARY_COEFFICIENTS = new PIDFCoefficients(PRIMARY_KP, PRIMARY_KI, PRIMARY_KD, PRIMARY_KF);
    private final PIDFCoefficients SECONDARY_COEFFICIENTS = new PIDFCoefficients(SECONDARY_KP, SECONDARY_KI, SECONDARY_KD, SECONDARY_KF);
    private final RobotState robotState;
    private final DcMotor leftSlideMotor;
    private final DcMotor rightSlideMotor;
    private final DcMotor encoderMotor;
    private final DcMotorEx currentMotorRight;
    private final DcMotorEx currentMotorLeft;
    private final Servo leftWrist;
    private final Servo rightWrist;
    private final Servo claw;
    private final Servo clawRotation;
    private final SlideController slideController;
    private final DifferentialController differentialController;
    private final SlidingAverageCalculator leftSlideCurrentAverage;
    private final SlidingAverageCalculator rightSlideCurrentAverage;
    private boolean isDirectControlEnabled;
    private final DigitalChannel breakBeamSensor;
    private boolean inPrimarySlideMode;

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
        claw = hardwareMap.get(Servo.class, "intake_claw");
        //Claw rotation zero is perpendicular to the slides, the triangle facing forwards
        clawRotation = hardwareMap.get(Servo.class, "intake_claw_rotation");
        breakBeamSensor = hardwareMap.get(DigitalChannel.class, "intake_break_beam");

        slideController = new SlideController(MOTOR_TICKS_PER_INCH, PRIMARY_COEFFICIENTS);
        inPrimarySlideMode = true;
        differentialController = new DifferentialController(DIFFERENTIAL_GEAR_RATIO, 270, SERVO_GEAR_RATIO);//TODO: Find max servo angle
        differentialController.setMaxRange(180, 180);
        rightSlideCurrentAverage = new SlidingAverageCalculator(10);
        leftSlideCurrentAverage = new SlidingAverageCalculator(10);

        breakBeamSensor.setMode(DigitalChannel.Mode.INPUT);

        //Assuming that the encoder is connected to the leftSlideMotor
        encoderMotor = rightSlideMotor;
        currentMotorRight = (DcMotorEx) rightSlideMotor;
        currentMotorLeft = (DcMotorEx) leftSlideMotor;

        //Configure Motors
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        rightSlideMotor.setDirection(DcMotor.Direction.REVERSE);

        rightWrist.setDirection(Servo.Direction.REVERSE);
        leftWrist.setDirection(Servo.Direction.FORWARD);

        claw.setDirection(Servo.Direction.REVERSE);
        clawRotation.setDirection(Servo.Direction.REVERSE);
        isDirectControlEnabled = false;

        if (robotState.isAuto()) {
            init();
            resetSlides();
        }

        moveSlidesAbsolute(getCurrentSlidePositionInches());

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
    public double getWristPitch() {
        return differentialController.getPitchAndRotation(leftWrist.getPosition(), rightWrist.getPosition())[0];
    }

    /**
     * @return the rotation of the wrist in degrees
     */
    public double getWristRotation() {
        return differentialController.getPitchAndRotation(leftWrist.getPosition(), rightWrist.getPosition())[1];
    }

    /**
     * @return the position of the claw
     */
    public double getClawPosition() {
        return claw.getPosition();
    }

    /**
     * @return the rotation of the claw in degrees
     */
    public double getClawRotation() {
        return clawRotation.getPosition() * INTAKE_CLAW_ROTATION_RANGE - CLAW_ROTATION_BUFFER;
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
     * Moves the Slides relatively, but with no restriction on movement
     *
     * @param distance The distance you want to move in inches
     */
    public void moveSlidesRelativeUnsafe(double distance) {
        slideController.moveToInches(getCurrentSlidePositionInches() + distance);
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
        return slideController.getTargetTicks() / MOTOR_TICKS_PER_INCH;
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
        claw.setPosition(CLAW_OPEN_POSITION);
        robotState.setIntakeClawState(ClawState.OPEN);
    }

    /**
     * Closes the Intake Claw
     */
    public void closeClaw() {
        claw.setPosition(CLAW_CLOSED_POSITION);
        robotState.setIntakeClawState(ClawState.CLOSED);
    }

    /**
     * Loosens the intake claw
     */
    public void loosenClaw() {
        claw.setPosition(CLAW_LOOSE_POSITION);
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
        setWristAbsolute(getWristPitch() + pitchAngle, getWristRotation() + rotationAngle);
    }

    /**
     * Sets the pitch of the wrist, while keeping the rotation the same
     *
     * @param pitchAngle the desired pitch of the wrist
     */
    public void setWristPitchAbsolute(double pitchAngle) {
        setWristAbsolute(pitchAngle, getWristRotation());
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
        setWristAbsolute(getWristPitch(), rotationAngle);
    }

    /**
     * Toggles the rotation of the wrist between 0 and 90
     */
    public void togglePerpendicularRotation() {
        if (Math.abs(getClawRotation() - CLAW_ROTATION_PICKUP_POSITION) < 2) {
            setClawRotationAbsolute(180);
        } else {
            setClawRotationAbsolute(CLAW_ROTATION_PICKUP_POSITION);
        }
    }

    /**
     * Sets the rotation of the claw.
     *
     * @param rotationAngle the desired rotation of the claw in degrees from 0º to 180º
     */
    public void setClawRotationAbsolute(double rotationAngle) {
        clawRotation.setPosition(Range.clip(rotationAngle + CLAW_ROTATION_BUFFER,
                CLAW_ROTATION_BUFFER, CLAW_ROTATION_BUFFER + 180) / INTAKE_CLAW_ROTATION_RANGE);
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
    public double getSlideCurrentLeft() {
        return leftSlideCurrentAverage.getAverage();
    }

    /**
     * Updates the block position of the robot
     */
    private void updateBlockPosition() {
        if (isBlockInIntake()) {
            robotState.setBlockPosition(RobotBlockPosition.INTAKE);
        } else if (robotState.getBlockPosition() == RobotBlockPosition.INTAKE) {
            robotState.setBlockPosition(RobotBlockPosition.NONE);
        }
    }

    /**
     * @return whether or not the block is in the intake
     */
    public boolean isBlockInIntake() {
        return !breakBeamSensor.getState() && robotState.getIntakeClawState() == ClawState.CLOSED;
    }

    /**
     * Sets the direct control motor mode
     *
     * @param directControlEnabled boolean to set the direct control to
     */
    public void setDirectControl(boolean directControlEnabled) {
        isDirectControlEnabled = directControlEnabled;
    }

    /**
     * Sets both slide motors to a given power, also using voltage to compensate for the correct power
     *
     * @param power the given power to set the motors to
     */
    public void setMotorPower(double power) {
        if (isDirectControlEnabled) {
            if ((getCurrentSlidePositionInches() > SLIDES_MAX && power > 0) || (getCurrentSlidePositionInches() < 0 && power < 0)) {
                power = 0;
            }
            power = getVoltageCompensatedMotorPower(power);
            leftSlideMotor.setPower(power);
            rightSlideMotor.setPower(power);
        }
    }

    private double getVoltageCompensatedMotorPower(double power) {
        if (robotState.getVoltage() != 0) {
            return Range.clip(power / (robotState.getVoltage() / 12.0), -1, 1);
        } else {
            return power;
        }
    }

    /**
     * Updates and powers motors every cycle
     */
    @Override
    public void periodic() {
        if (!isDirectControlEnabled) {
            if(getCurrentSlidePositionInches() > 15 && inPrimarySlideMode) {
                slideController.setPIDFCoefficients(SECONDARY_COEFFICIENTS);
                inPrimarySlideMode = false;
            } else if (getCurrentSlidePositionInches() <= 15 && !inPrimarySlideMode) {
                slideController.setPIDFCoefficients(PRIMARY_COEFFICIENTS);
                inPrimarySlideMode = true;
            }
            double power = getVoltageCompensatedMotorPower(slideController.calculateMotorPowers(encoderMotor.getCurrentPosition()));
            leftSlideMotor.setPower(power);
            rightSlideMotor.setPower(power);
        }

        robotState.setHorizontalExtended(encoderMotor.getCurrentPosition() > 100);
        double[] wristAngles = differentialController.getPitchAndRotation(leftWrist.getPosition(), rightWrist.getPosition());
        double[] wristPositions = differentialController.calculateServoPositions(wristAngles[0], wristAngles[1]);

        leftSlideCurrentAverage.add(currentMotorLeft.getCurrent(CurrentUnit.AMPS));
        rightSlideCurrentAverage.add(currentMotorRight.getCurrent(CurrentUnit.AMPS));

        robotState.setIntakeSlidePosition(getCurrentSlidePositionInches());

        robotState.setIntakeCurrent(rightSlideCurrentAverage.getAverage() + leftSlideCurrentAverage.getAverage());

        if(robotState.getIntakeState() == IntakeState.READY_TO_PICKUP) {
            updateBlockPosition();
        }
    }
}
