package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.button.Trigger;
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
        FtcDashboard dashboard = FtcDashboard.getInstance();

        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        RobotState robotState = new RobotState();
        intakeSubsystem = new IntakeSubsystem(hardwareMap, robotState);
        registerSubsystems(intakeSubsystem);

        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            intakeSubsystem.openClaw();
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            intakeSubsystem.closeClaw();
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            intakeSubsystem.setWristRelative(-5, 0);
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            intakeSubsystem.setWristRelative(5, 0);
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
            intakeSubsystem.setWristRelative(0, -5);
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            intakeSubsystem.setWristRelative(0, 5);
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            intakeSubsystem.setWristAbsolute(90, 90);
        });
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            intakeSubsystem.setWristAbsolute(180, 90);
        });

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> {
            intakeSubsystem.setClawRotationRelative(5);
        });

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> {
            intakeSubsystem.setClawRotationRelative(-5);
        });


        Trigger slidesTrigger = new Trigger(() ->
                gamepadEx.getLeftY() != 0
        );

        slidesTrigger.whileActiveContinuous(() -> intakeSubsystem.moveSlidesRelative(gamepadEx.getLeftY() * 2));
    }

    @Override
    public void update() {
        telemetry.addData("Claw rotation angle", intakeSubsystem.getClawRotation());
        telemetry.addData("Claw diff pitch", intakeSubsystem.getPitch());
        telemetry.addData("Claw diff rotation", intakeSubsystem.getRotation());
        telemetry.addData("Slides Position", intakeSubsystem.getCurrentSlidePositionInches());
    }
}
