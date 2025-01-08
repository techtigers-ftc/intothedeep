package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedropathing_old.DriveVectors;
import org.firstinspires.ftc.teamcode.pedropathing_old.follower.DriveVectorScaler;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.SlidingAverageCalculator;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem that controls the drivebase.
 */
public class DriveSubsystem extends CloseableSubsystem {
    private static final double GEAR_MULTIPLIER = 0.5;
    private static final double TURN_MULTIPLIER = 0.75;
    private static final double TURN_GEAR_MULTIPLIER = 0.375;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final DriveVectorScaler driveVectorScaler;
    private final RobotState robotstate;
    private final SlidingAverageCalculator frontLeftSlideCurrentAverage;
    private final SlidingAverageCalculator frontRightSlideCurrentAverage;
    private final SlidingAverageCalculator backLeftSlideCurrentAverage;
    private final SlidingAverageCalculator backRightSlideCurrentAverage;
    private final DcMotorEx currentFrontRight;
    private final DcMotorEx currentFrontLeft;
    private final DcMotorEx currentBackRight;
    private final DcMotorEx currentBackLeft;

    /**
     * Constructs a new DriveSubsystem.
     *
     * @param hardwareMap The hardware map, used to get hardware references
     */
    public DriveSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front");
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        backRight = hardwareMap.get(DcMotor.class, "right_back");

        frontRightSlideCurrentAverage = new SlidingAverageCalculator(10);
        frontLeftSlideCurrentAverage = new SlidingAverageCalculator(10);
        backRightSlideCurrentAverage = new SlidingAverageCalculator(10);
        backLeftSlideCurrentAverage = new SlidingAverageCalculator(10);
        currentFrontRight = (DcMotorEx) frontRight;
        currentFrontLeft = (DcMotorEx) frontLeft;
        currentBackRight = (DcMotorEx) backRight;
        currentBackLeft = (DcMotorEx) backLeft;

        driveVectorScaler = new DriveVectorScaler(FollowerConstants.frontLeftVector);

        DcMotor[] motors = {frontLeft, backLeft, frontRight, backRight};
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        this.robotstate = robotState;

        for (DcMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    //From FTC Lib RobotDrive
    private void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }
    }

    //From FTC Lib RobotDrive
    private void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }
    }

    /**
     * Drives the robot in robot centric mode, with movement inputs relative to the robot's orientation.
     *
     * @param forward  The forward power
     * @param strafe   The strafe power
     * @param rotation The rotation power
     */
    public void driveRobotCentric(double forward, double strafe, double rotation) {
        driveFieldCentric(forward, strafe, rotation, 0.0);
    }

    /**
     * Drives the robot in field centric mode, with movement inputs relative to the field's orientation.
     *
     * @param forward  The forward power
     * @param strafe   The strafe power
     * @param rotation The rotation power
     * @param heading  The robot's heading
     */
    public void driveFieldCentric(double forward, double strafe, double rotation, double heading) {
        RobotLog.dd(tag, "----------------------------------");
        RobotLog.dd(tag, "Forward: %f, Strafe: %f, Turn: %f",
                forward, strafe, rotation);
        double strafeSpeed = Range.clip(strafe, -1, 1);
        double forwardSpeed = Range.clip(forward, -1, 1);
        double turnSpeed = Range.clip(rotation, -1, 1) * TURN_MULTIPLIER;

        if (robotstate.getCurrentGear() == DriveGears.ENGAGED) {
            strafeSpeed *= GEAR_MULTIPLIER;
            forwardSpeed *= GEAR_MULTIPLIER;
            turnSpeed *= TURN_GEAR_MULTIPLIER; // This is intended to be on top of the other multiplier
        }

        RobotLog.dd(tag, "Forward: %f, Strafe: %f, Turn: %f",
                forwardSpeed, strafeSpeed, turnSpeed);

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-heading);

        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        //Front Left
        wheelSpeeds[0] = Math.sin(theta + Math.PI / 4);
        //Front Right
        wheelSpeeds[2] = Math.sin(theta - Math.PI / 4);
        //Back Left
        wheelSpeeds[1] = Math.sin(theta - Math.PI / 4);
        //Back Right
        wheelSpeeds[3] = Math.sin(theta + Math.PI / 4);
        RobotLog.dd(tag, "FL: %f, BL: %f, FR: %f, BR: %f",
                wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);

        normalize(wheelSpeeds, input.magnitude());
        RobotLog.dd(tag, "FL: %f, BL: %f, FR: %f, BR: %f",
                wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);

        wheelSpeeds[0] += turnSpeed;
        wheelSpeeds[2] -= turnSpeed;
        wheelSpeeds[1] += turnSpeed;
        wheelSpeeds[3] -= turnSpeed;
        RobotLog.dd(tag, "FL: %f, BL: %f, FR: %f, BR: %f",
                wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);

        normalize(wheelSpeeds);
        RobotLog.dd(tag, "FL: %f, BL: %f, FR: %f, BR: %f",
                wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);

        setMotorPowers(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
    }

    /**
     * Private method to set the motor powers.
     *
     * @param fl The front left motor power
     * @param fr The front right motor power
     * @param bl The back left motor power
     * @param br The back right motor power
     */
    private void setMotorPowers(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void toggleDriveGears() {
        if (robotstate.getCurrentGear() == DriveGears.ENGAGED) {
            robotstate.setCurrentGear(DriveGears.NOT_ENGAGED);
        } else {
            robotstate.setCurrentGear(DriveGears.ENGAGED);
        }
    }

    /**
     * Drives the robot given the vectors calculated by the pedro path follower
     *
     * @param vectors The vectors to drive with
     */
    public void drivePedroPath(DriveVectors vectors) {
        double[] drivePowers = driveVectorScaler.getDrivePowers(vectors.correctivePower, vectors.headingPower, vectors.pathingPower, vectors.robotHeading);
        setMotorPowers(drivePowers[0], drivePowers[1], drivePowers[2], drivePowers[3]);
    }

    @Override
    public void periodic() {
        frontLeftSlideCurrentAverage.add(currentFrontLeft.getCurrent(CurrentUnit.AMPS));
        frontRightSlideCurrentAverage.add(currentFrontRight.getCurrent(CurrentUnit.AMPS));
        backLeftSlideCurrentAverage.add(currentBackLeft.getCurrent(CurrentUnit.AMPS));
        backRightSlideCurrentAverage.add(currentBackRight.getCurrent(CurrentUnit.AMPS));

        robotstate.setDriverCurrent(frontLeftSlideCurrentAverage.getAverage() + frontRightSlideCurrentAverage.getAverage() + backLeftSlideCurrentAverage.getAverage() + backRightSlideCurrentAverage.getAverage());

        RobotLog.dd(tag, "Front Left Current: %f, Front Right Current: %f, Back Left Current: %f, Back Right Current: %f",
                frontLeftSlideCurrentAverage.getAverage(), frontRightSlideCurrentAverage.getAverage(), backLeftSlideCurrentAverage.getAverage(), backRightSlideCurrentAverage.getAverage());
    }
}
