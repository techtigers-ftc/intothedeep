package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.CloseableSubsystem;

public class AscentSubsystem extends CloseableSubsystem {
    private final Servo changingTransmission;
    private final DcMotor leftVerticalSlide;
    private final DcMotor rightVerticalSlide;
    private final DcMotor leftBack;
    private final DcMotor rightBack;
    private final RobotState robotState;

    public AscentSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        changingTransmission = hardwareMap.get(Servo.class,
                "transmission_switch");
        leftVerticalSlide = hardwareMap.get(DcMotor.class, "left_dropper_slide");
        rightVerticalSlide = hardwareMap.get(DcMotor.class, "right_dropper_slide");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftVerticalSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightVerticalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        disengageAscent();
    }

    public void engageAscent() {
        changingTransmission.setPosition(0.61);
        robotState.setIsAscending(true);
    }

    public void disengageAscent() {
        changingTransmission.setPosition(0.5);
        robotState.setIsAscending(false);
    }

    public void setChangingTransmissionPosition(double position) {
        changingTransmission.setPosition(position);
    }

    public double getChangingTransmissionPosition() {
        return changingTransmission.getPosition();
    }

    public void powerAscent(double power) {
        leftVerticalSlide.setPower(power);
        rightVerticalSlide.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    @Override
    public void periodic(){
        RobotLog.dd("AscentSubsystem", "Changing Transmission Position: %f", getChangingTransmissionPosition());
    }

    @Override
    public void close() {
        disengageAscent();
    }
}
