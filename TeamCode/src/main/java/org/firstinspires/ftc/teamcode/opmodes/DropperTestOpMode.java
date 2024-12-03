package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

/**
 * An opmode to test the capabilities of the dropper subsystem, including the slides, arm, and claw
 */
@TeleOp(name = "Dropper Test OpMode")
public class DropperTestOpMode extends BaseOpMode {
    private DropperSubsystem dropperSubsystem;
    private RobotState robotState;
    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        driverGamepad = new GamepadEx(gamepad1);
        dropperSubsystem = new DropperSubsystem(hardwareMap, robotState);
        registerSubsystems(dropperSubsystem);
        dropperSubsystem.resetSlides();

        // Moving by 1-inch increments
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.moveSlidesRelative(1);
        }));

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.moveSlidesRelative(-1);
        }));

        // Moving by 5-inch increments
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.moveSlidesRelative(-5);
        }));

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.moveSlidesRelative(5);
        }));

        // Emergency stop button for the slides
        driverGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.stopSlides();
        }));

        // Moves slides to their near-max and near-min height
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.moveSlidesAbsoluteInches(28);
        }));

        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.moveSlidesAbsoluteInches(1);
        }));

        // Manual control of the slides
        Trigger joystickTrigger = new Trigger(() -> driverGamepad.getLeftY() != 0);
        joystickTrigger.whileActiveContinuous(() -> dropperSubsystem.moveSlidesRelative(driverGamepad.getLeftY() * 2));
    }

    @Override
    public void update() {
        double currentPos = dropperSubsystem.getCurrentPositionInInches();
        double expectedPos = dropperSubsystem.getTargetPositionInches();
        double error = expectedPos - currentPos;

        telemetry.addData("CurrentPosInches", currentPos);
        telemetry.addData("ExpectedPosInches", expectedPos);
        telemetry.addData("Error", error);
    }
}
