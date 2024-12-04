package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp
public class DropperArmTestOpmode extends BaseOpMode {
    private DropperSubsystem dropperSubsystem;
    private RobotState robotState;
    private GamepadEx gamepadEx;

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        dropperSubsystem = new DropperSubsystem(hardwareMap, robotState);
        registerSubsystems(dropperSubsystem);
        gamepadEx = new GamepadEx(gamepad1);

       dropperSubsystem.setPitchAbsolute(0.5);

       // Claw
       gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
           dropperSubsystem.openClaw();
        }));

        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.closeClaw();
        }));

        // Pitch

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.setPitchRelative(-0.05);
        }));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.setPitchRelative(0.05);
        }));

        gamepadEx.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.setPitchAbsolute(0.5);
        }));

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.setPitchAbsolute(1);
        }));

        // Rotation

        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.rotateClawDown();
        }));

        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.rotateClawUp();
        }));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.setRotationRelative(-0.05);
        }));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.setRotationRelative(0.05);
        }));
    }

    @Override
    public void update(){
        telemetry.addData("Pitch Servo Value", dropperSubsystem.getPitchPos());
        telemetry.addData("Rotation Servo Value", dropperSubsystem.getRotationPos());
        telemetry.addData("Claw Servo Value", dropperSubsystem.getClawPos());
    }
}
