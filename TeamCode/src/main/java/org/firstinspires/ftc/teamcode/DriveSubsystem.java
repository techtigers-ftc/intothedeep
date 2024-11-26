package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

import java.util.Arrays;
import java.util.List;

import team.techtigers.base.CloseableSubsytem;

/**
 * A subsystem that controls the drivebase.
 */
public class DriveSubsystem extends CloseableSubsytem {
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final List<DcMotorEx> motors;

    /**
     * Constructs a new DriveSubsystem.
     *
     * @param hardwareMap The hardware map, used to get hardware references
     */
    public DriveSubsystem(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "left_front");
        frontRight = hardwareMap.get(DcMotorEx.class, "right_front");
        backLeft = hardwareMap.get(DcMotorEx.class, "left_back");
        backRight = hardwareMap.get(DcMotorEx.class, "right_back");

        motors = Arrays.asList(frontLeft, backLeft, frontRight, backRight);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }


    /**
     * Private method to set the motor powers.
     *
     * @param fl The front left motor power
     * @param fr The front right motor power
     * @param bl The back left motor power
     * @param br The back right motor power
     */
    public void setMotorPowers(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    /**
     * Drives the robot based on the values for the motors from the follower in pedro pathing
     *
     * @param follower the follower to get values from
     */
    public void driveFollower(Follower follower) {
        double[] drivePowers = follower.getDrivePowers();
        setMotorPowers(drivePowers[0], drivePowers[1], drivePowers[2], drivePowers[3]);
    }
}
