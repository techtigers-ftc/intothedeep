package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import team.techtigers.base.CloseableSubsystem;

public class AscentSubsystem extends CloseableSubsystem {
    private final Servo changingTransmission;
    private final DcMotor leftVerticalSlide;
    private final DcMotor rightVerticalSlide;
    private final DcMotor leftBack;
    private final DcMotor rightBack;


    public AscentSubsystem(HardwareMap hardwareMap) {
        changingTransmission = hardwareMap.get(Servo.class, "transmission_switch");
        leftVerticalSlide = hardwareMap.get(DcMotor.class, "left_dropper_slide");
        rightVerticalSlide = hardwareMap.get(DcMotor.class, "right_dropper_slide");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
    }

    public void setChangingTransmissionPosition(double position) {
        changingTransmission.setPosition(position);
    }

    public double getChangingTransmissionPosition() {
        return changingTransmission.getPosition();
    }

    @Override
    public void periodic(){
        RobotLog.dd("AscentSubsystem", "Changing Transmission Position: %f", getChangingTransmissionPosition());
    }
}
