package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryWallAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

import team.techtigers.base.BaseOpMode;

/**
 * An opmode to test the capabilities of the dropper subsystem, including the slides, arm, and claw
 */
@TeleOp(name = "Dropper Tuning OpMode", group = "Tuning")
public class DropperTuningOpMode extends BaseOpMode {
    private DropperSubsystem dropperSubsystem;
    private RobotState robotState;
    private IntakeSubsystem intakeSubsystem;
    private DriveSubsystem drive;

    @Override
    public void initialize() {
        robotState = new RobotState(true, false);
        dropperSubsystem = new DropperSubsystem(hardwareMap, robotState);
        registerSubsystems(dropperSubsystem);

        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Claw
        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.closeClaw();
        }));
        driverGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.openClaw();
        }));

        // Pitch
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.setPitchRelative(-5);
        }));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.setPitchRelative(5);
        }));

        // Rotation
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.setRotationRelative(-5);
        }));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.setRotationRelative(5);
        }));

        // Presets for the differential
        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.setPitchAbsolute(355);
        }));
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.setPitchAbsolute(0);
        }));

        // Slides
        Trigger dropperSlidesTrigger = new Trigger(() ->
                driverGamepad.getRightY() != 0
        );
        dropperSlidesTrigger.whileActiveContinuous(() ->
                dropperSubsystem.moveSlidesRelative(
                        -driverGamepad.getRightY() * 2.5)
        );

        // Emergency stop button for the slides
        driverGamepad.getGamepadButton(GamepadKeys.Button.START).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.stopSlides();
        }));
    }

    @Override
    public void update() {
        double currentPos = dropperSubsystem.getCurrentSlidePositionInches();
        double expectedPos = dropperSubsystem.getTargetPositionInches();
        double error = expectedPos - currentPos;

        telemetry.addData("Current slide position (inches)", currentPos);
        telemetry.addData("Expected slide position (inches)", expectedPos);
        telemetry.addData("Slide position error (inches)", error);
        telemetry.addLine();
        telemetry.addData("Claw rotation angle", dropperSubsystem.getRotation());
        telemetry.addData("Claw diff pitch", dropperSubsystem.getPitch());
        telemetry.addLine();
        telemetry.addData("Left slide current draw:", dropperSubsystem.getSlideCurrentLeft());
        telemetry.addData("Right slide current draw:", dropperSubsystem.getSlideCurrentRight());
        telemetry.addData("Invalid dropper state error:", robotState.hasError(RobotError.INVALID_DROPPER_POSITION));
    }
}
