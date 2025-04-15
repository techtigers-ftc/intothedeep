package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

/**
 * An opmode to test the capabilities of the robot pitch from the imu
 */
@TeleOp(name = "Robot Pitch Test OpMode", group = "Tuning")
public class RobotPitchOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        robotState = new RobotState(true, false);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);
        registerSubsystems(sensor);
    }

    @Override
    public void update() {
        telemetry.addData("Robot Pitch", robotState.getRobotPitch());
    }
}
