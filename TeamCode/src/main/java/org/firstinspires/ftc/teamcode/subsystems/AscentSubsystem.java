package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import team.techtigers.base.CloseableSubsystem;

public class AscentSubsystem extends CloseableSubsystem {
    private final Servo changingTransmission;

    public AscentSubsystem(HardwareMap hardwareMap) {
        changingTransmission = hardwareMap.get(Servo.class, "changingTransmission");
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
