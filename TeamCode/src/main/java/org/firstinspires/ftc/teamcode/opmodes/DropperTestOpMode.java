package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp (name = "Dropper Test OpMode")
public class DropperTestOpMode extends BaseOpMode {
    private DropperSubsystem dropperSubsystem;
    private RobotState robotState;
    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        dropperSubsystem = new DropperSubsystem(hardwareMap, robotState);
        registerSubsystems(dropperSubsystem);

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.moveSlidesRelative(1);
        }));

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.moveSlidesRelative(-1);
        }));
    }
}
