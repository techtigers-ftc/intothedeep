package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

import team.techtigers.base.BaseOpMode;

@TeleOp(name = "Dropper Slides Tuning", group = "Dropper Tuning")
@Disabled
public class DropperSlidesTuningOpmode extends BaseOpMode {
    private DropperSubsystem dropperSubsystem;
    private RobotState robotState;

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        robotState = new RobotState(true, false);
        dropperSubsystem = new DropperSubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);
        registerSubsystems(dropperSubsystem, sensor);

        // Slides
        Trigger slidesTrigger = new Trigger(() ->
                driverGamepad.getLeftY() != 0
        );
        slidesTrigger.whileActiveContinuous(() -> dropperSubsystem.moveSlidesRelative(driverGamepad.getLeftY() * 2));

        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            dropperSubsystem.moveSlidesAbsolute(0);
        });

        driverGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            dropperSubsystem.moveSlidesAbsolute(6);
        });

        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            dropperSubsystem.moveSlidesAbsolute(12);
        });

        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            dropperSubsystem.moveSlidesAbsolute(DropperSubsystem.SLIDE_MAX);
        });

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
        telemetry.addData("Left slide motor current", dropperSubsystem.getSlideCurrentLeft());
        telemetry.addData("Right slide motor current", dropperSubsystem.getSlideCurrentRight());
        telemetry.addData("Invalid intake error:", robotState.hasError(RobotError.INVALID_INTAKE_POSITION));
    }
}
