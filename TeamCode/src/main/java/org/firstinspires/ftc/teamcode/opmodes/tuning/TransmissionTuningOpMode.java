package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

import team.techtigers.base.BaseOpMode;

/**
 * An opmode to test the capabilities of the dropper subsystem, including the slides, arm, and claw
 */
@TeleOp(name = "Transmission Tuning OpMode", group = "Tuning")
public class TransmissionTuningOpMode extends BaseOpMode {
    private AscentSubsystem ascentSubsystem;
    private RobotState robotState;

    @Override
    public void initialize() {
        robotState = new RobotState(true, false);
        ascentSubsystem = new AscentSubsystem(hardwareMap, robotState);
        registerSubsystems(ascentSubsystem);

        GamepadEx driverGamepad = new GamepadEx(gamepad1);

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> {
            ascentSubsystem.setChangingTransmissionPosition(ascentSubsystem.getChangingTransmissionPosition()-0.01);
        }));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> {
            ascentSubsystem.setChangingTransmissionPosition(ascentSubsystem.getChangingTransmissionPosition()+0.01);
        }));
    }

    @Override
    public void update() {
        telemetry.addData("Transmission Position", ascentSubsystem.getChangingTransmissionPosition());
    }
}
