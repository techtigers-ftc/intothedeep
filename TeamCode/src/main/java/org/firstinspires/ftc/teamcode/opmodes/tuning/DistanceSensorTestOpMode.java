package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp
public class DistanceSensorTestOpMode extends BaseOpMode {
    RobotState robotState;
    SensorSubsystem sensorSubsystem;


    public void initialize() {
        robotState = new RobotState(true, false);
        sensorSubsystem = new SensorSubsystem(hardwareMap, robotState);
        registerSubsystems(sensorSubsystem);
        robotState.setRunDistanceSensor(true);
    }

    @Override
    public void update() {
        telemetry.addData("Distance Sensor Value", robotState.getDistanceSensorValue());
    }
}
