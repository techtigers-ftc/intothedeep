package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DropperToHighBasketDropCommandGroup;
import org.firstinspires.ftc.teamcode.commands.DropperToHighChamberFromIntakeDropCommandGroup;
import org.firstinspires.ftc.teamcode.commands.DropperToHighChamberFromWallDropCommandGroup;
import org.firstinspires.ftc.teamcode.commands.DropperToTransferCommandGroup;
import org.firstinspires.ftc.teamcode.commands.DropperToWallIntakeCommandGroup;
import org.firstinspires.ftc.teamcode.commands.IntakePitchTriggerRotateCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeToIntakeCommandGroup;
import org.firstinspires.ftc.teamcode.commands.IntakeToTransferCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp(name = "Scrimmage TeleOp Mode", group = "Scrimmage")
public class ScrimmageTeleOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        RobotState robotState = new RobotState();
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap, robotState);
        registerSubsystems(intake);

        IntakeToIntakeCommandGroup intakeToIntake =
                new IntakeToIntakeCommandGroup(intake, robotState);
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(intakeToIntake);

        IntakeToTransferCommandGroup intakeToTransfer =
                new IntakeToTransferCommandGroup(intake, robotState);
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(intakeToTransfer);

        DropperToWallIntakeCommandGroup dropperToWall =
                new DropperToWallIntakeCommandGroup(dropper, robotState);
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(dropperToWall);

        DropperToTransferCommandGroup dropperToTransfer =
                new DropperToTransferCommandGroup(dropper, robotState);
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(dropperToTransfer);


        DropperToHighBasketDropCommandGroup highBasketDrop =
                new DropperToHighBasketDropCommandGroup(dropper, intake, robotState);
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(highBasketDrop);


        DropperToHighChamberFromIntakeDropCommandGroup highChamberDropFromIntake =
                new DropperToHighChamberFromIntakeDropCommandGroup(dropper, intake, robotState);
        DropperToHighChamberFromWallDropCommandGroup highChamberDropFromWall =
                new DropperToHighChamberFromWallDropCommandGroup(dropper, robotState);
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
                    if (robotState.isIntakeFromWall()) {
                        highChamberDropFromWall.schedule();
                    } else {
                        highChamberDropFromIntake.schedule();
                    }
                }
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                () -> intake.moveSlidesRelative(0)
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(
                () -> intake.setWristAbsolute(180, 90),
                () -> intake.setWristAbsolute(180, 0)
        );


        gamepadEx.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
                intake::closeClaw,
                intake::openClaw
        );

        gamepadEx.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(
                dropper::closeClaw,
                dropper::openClaw
        );


        Trigger intakeSlidesTrigger = new Trigger(() ->
                gamepadEx.getLeftY() != 0
        );
        intakeSlidesTrigger.whileActiveContinuous(() -> intake.moveSlidesRelative(
                gamepadEx.getLeftY() * 2));

        Trigger dropperSlidesTrigger = new Trigger(() ->
                gamepadEx.getRightY() != 0
        );
        dropperSlidesTrigger.whileActiveContinuous(() -> dropper.moveSlidesRelative(
                gamepadEx.getRightY() * 2));

        IntakePitchTriggerRotateCommand intakePitchTriggerRotateCommand =
                new IntakePitchTriggerRotateCommand(intake, gamepadEx);
        Trigger intakeRotationTrigger = new Trigger(() ->
                gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 ||
                        gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0
        );
        intakeRotationTrigger.whileActiveContinuous(intakePitchTriggerRotateCommand);

    }
}
