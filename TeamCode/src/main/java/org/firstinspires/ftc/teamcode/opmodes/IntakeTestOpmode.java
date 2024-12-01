package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;
@TeleOp
public class IntakeTestOpmode extends BaseOpMode {
    private IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        RobotState robotState = new RobotState();
        intakeSubsystem = new IntakeSubsystem(hardwareMap, robotState);
        registerSubsystems(intakeSubsystem);

        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(() ->{
            intakeSubsystem.closeClaw();
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(() ->{
           intakeSubsystem.openClaw();
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() ->{
            intakeSubsystem.setWristRelative(-5,0);
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() ->{
            intakeSubsystem.setWristRelative(5,0);
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() ->{
            intakeSubsystem.setWristRelative(0,-5);
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() ->{
            intakeSubsystem.setWristRelative(0,5);
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.X).whenPressed(() ->{
            intakeSubsystem.setWristAbsolute(90,90);
        });
    }

    @Override
    public void update(){
        intakeSubsystem.powerSlides(gamepad1.left_stick_y);
        telemetry.addData("Position", intakeSubsystem.getCurrentPositionInches());
    }
}
