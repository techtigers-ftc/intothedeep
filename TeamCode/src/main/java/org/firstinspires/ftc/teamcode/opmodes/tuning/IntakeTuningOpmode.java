package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

import team.techtigers.base.BaseOpMode;

@TeleOp(name = "Intake Tuning OpMode", group = "Tuning")
public class IntakeTuningOpmode extends BaseOpMode {
    private IntakeSubsystem intakeSubsystem;
    private RobotState robotState;

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        robotState = new RobotState();
        intakeSubsystem = new IntakeSubsystem(hardwareMap, robotState);
        registerSubsystems(intakeSubsystem);

        // Claw
        driverGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            intakeSubsystem.openClaw();
        });
        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            intakeSubsystem.closeClaw();
        });

        // Wrist Pitch
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            intakeSubsystem.setWristRelative(-5, 0);
        });
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            intakeSubsystem.setWristRelative(5, 0);
        });

        // Wrist Rotation
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
            intakeSubsystem.setWristRelative(0, -5);
        });
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            intakeSubsystem.setWristRelative(0, 5);
        });

        // Presets for the differential
        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            intakeSubsystem.setWristAbsolute(90, 90);
        });
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            intakeSubsystem.setWristAbsolute(180, 90);
        });

        // Claw Rotation
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> {
            intakeSubsystem.setClawRotationRelative(5);
        });
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> {
            intakeSubsystem.setClawRotationRelative(-5);
        });

        // Slides
        Trigger slidesTrigger = new Trigger(() ->
                driverGamepad.getLeftY() != 0
        );
        slidesTrigger.whileActiveContinuous(() -> intakeSubsystem.moveSlidesRelative(driverGamepad.getLeftY() * 2));

        // Emergency stop button for the slides
        driverGamepad.getGamepadButton(GamepadKeys.Button.START).whenPressed(new InstantCommand(() -> {
            intakeSubsystem.stopSlides();
        }));
    }

    @Override
    public void update() {
        double currentPos = intakeSubsystem.getCurrentSlidePositionInches();
        double expectedPos = intakeSubsystem.getTargetPositionInches();
        double error = expectedPos - currentPos;

        telemetry.addData("Current slide position (inches)", currentPos);
        telemetry.addData("Expected slide position (inches)", expectedPos);
        telemetry.addData("Slide position error (inches)", error);
        telemetry.addLine();
        telemetry.addData("Claw rotation angle", intakeSubsystem.getClawRotation());
        telemetry.addData("Claw diff pitch", intakeSubsystem.getPitch());
        telemetry.addData("Claw diff rotation", intakeSubsystem.getRotation());
        telemetry.addData("Slides position", intakeSubsystem.getCurrentSlidePositionInches());
        telemetry.addLine();
        telemetry.addData("Current Draw", intakeSubsystem.getSlideMotorCurrent());
        telemetry.addData("Invalid intake error:", robotState.hasError(RobotError.INVALID_INTAKE_POSITION));
    }
}
