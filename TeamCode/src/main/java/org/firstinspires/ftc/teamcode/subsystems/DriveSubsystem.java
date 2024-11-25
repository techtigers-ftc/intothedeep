package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import team.techtigers.base.CloseableSubsytem;

/**
 * A subsystem that controls the drivebase.
 */
public class DriveSubsystem extends CloseableSubsytem {
    private final MecanumDrive drive;
    private final MotorEx frontLeft, frontRight, backLeft, backRight;

    /**
     * Constructs a new DriveSubsystem.
     *
     * @param hardwareMap The hardware map, used to get hardware references
     */
    public DriveSubsystem(HardwareMap hardwareMap) {
        frontLeft = new MotorEx(hardwareMap, "left_front");
        frontRight = new MotorEx(hardwareMap, "right_front");
        backLeft = new MotorEx(hardwareMap, "left_back");
        backRight = new MotorEx(hardwareMap, "right_back");

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeft.setInverted(true);
        frontRight.setInverted(true);
        backLeft.setInverted(true);
        backRight.setInverted(true);

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }

    /**
     * Private method to set the motor powers.
     *
     * @param fl The front left motor power
     * @param fr The front right motor power
     * @param bl The back left motor power
     * @param br The back right motor power
     */
    public void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.set(fl);
        frontRight.set(fr);
        backLeft.set(bl);
        backRight.set(br);
    }

    /**
     * Drives the robot with tele-op controls.
     *
     * @param forward  Forward power
     * @param strafe   Strafe power
     * @param rotation Rotation power
     */
    public void drive(double forward, double strafe, double rotation) {
        drive.driveRobotCentric(strafe, forward, rotation);
    }

    /**
     * Drives the robot with field centric controls.
     *
     * @param forward  Forward power
     * @param strafe   Strafe power
     * @param rotation Rotation power
     * @param heading  The heading of the robot
     */
    public void driveFieldCentric(double forward, double strafe, double rotation, double heading) {
        drive.driveFieldCentric(strafe, forward, rotation, heading);
    }
}
