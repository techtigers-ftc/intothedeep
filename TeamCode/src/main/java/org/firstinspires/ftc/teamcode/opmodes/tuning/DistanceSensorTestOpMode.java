package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.RawPowerToDistanceDriveAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
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
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        registerSubsystems(sensorSubsystem);
        robotState.setRunDistanceSensor(true);

        RawPowerToDistanceDriveAction distanceDriveAction = new RawPowerToDistanceDriveAction(drive, robotState, -0.5, 1.5);
        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(distanceDriveAction);
    }

    @Override
    public void update() {
        telemetry.addData("Distance Sensor Value", robotState.getDistanceSensorValue());
    }
}
