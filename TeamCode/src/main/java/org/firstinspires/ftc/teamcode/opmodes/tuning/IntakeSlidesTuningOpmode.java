package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

import team.techtigers.base.BaseOpMode;

@TeleOp(name = "Intake Slides Tuning", group = "Intake Tuning")
//@Disabled
public class IntakeSlidesTuningOpmode extends BaseOpMode {
    private IntakeSubsystem intakeSubsystem;
    private RobotState robotState;

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        robotState = new RobotState(true, false);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);
        registerSubsystems(intakeSubsystem, sensor);

        // Slides
        Trigger slidesTrigger = new Trigger(() ->
                driverGamepad.getLeftY() != 0
        );
        slidesTrigger.whileActiveContinuous(() -> intakeSubsystem.moveSlidesRelative(driverGamepad.getLeftY() * 2));

        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            intakeSubsystem.moveSlidesAbsolute(0);
        });

        driverGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            intakeSubsystem.moveSlidesAbsolute(6);
        });

        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            intakeSubsystem.moveSlidesAbsolute(12);
        });

        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            intakeSubsystem.moveSlidesAbsolute(IntakeSubsystem.SLIDES_MAX);
        });

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
        telemetry.addData("Left slide motor current", intakeSubsystem.getSlideCurrentLeft());
        telemetry.addData("Right slide motor current", intakeSubsystem.getSlideCurrentRight());
        telemetry.addData("Invalid intake error:", robotState.hasError(RobotError.INVALID_INTAKE_POSITION));
    }
}
