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
import org.firstinspires.ftc.teamcode.commands.HangSpecimenCommandGroup;
import org.firstinspires.ftc.teamcode.commands.IntakeManualRotationCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeToIntakeCommandGroup;
import org.firstinspires.ftc.teamcode.commands.IntakeToTransferCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import team.techtigers.base.BaseOpMode;

@TeleOp(name = "Scrimmage TeleOp Mode", group = "Scrimmage")
public class ScrimmageTeleOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        RobotState robotState = new RobotState();
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap, robotState);
        registerSubsystems(intake);

        IntakeToIntakeCommandGroup intakeToIntake =
                new IntakeToIntakeCommandGroup(intake, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(intakeToIntake);

        IntakeToTransferCommandGroup intakeToTransfer =
                new IntakeToTransferCommandGroup(intake, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(intakeToTransfer);

        DropperToWallIntakeCommandGroup dropperToWall =
                new DropperToWallIntakeCommandGroup(dropper, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(dropperToWall);

        DropperToTransferCommandGroup dropperToTransfer =
                new DropperToTransferCommandGroup(dropper, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(dropperToTransfer);

        Trigger intakeInTransfer = new Trigger(() ->
                robotState.getIntakeState() == IntakeState.TRANSFER
        );

        DropperToHighBasketDropCommandGroup highBasketDrop =
                new DropperToHighBasketDropCommandGroup(dropper, intake, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(intakeInTransfer).whenActive(highBasketDrop);

        Trigger intakeFromWall = new Trigger(robotState::isIntakeFromWall
        );

        DropperToHighChamberFromIntakeDropCommandGroup highChamberDropFromIntake =
                new DropperToHighChamberFromIntakeDropCommandGroup(dropper, intake, robotState);
        DropperToHighChamberFromWallDropCommandGroup highChamberDropFromWall =
                new DropperToHighChamberFromWallDropCommandGroup(dropper, robotState);

        // If the dropper is intaking a specimen from the wall, activate the high chamber drop from wall command
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).and(intakeFromWall).whenActive(highChamberDropFromWall);
        // If the dropper is transferring a specimen from the intake, activate the high chamber drop from intake command
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).and(intakeInTransfer).and(intakeFromWall.negate()).whenActive(highChamberDropFromIntake);

        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                () -> intake.moveSlidesRelative(0)
        );

        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(
                () -> intake.setWristAbsolute(180, 90),
                () -> intake.setWristAbsolute(180, 0)
        );


        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
                intake::closeClaw,
                intake::openClaw
        );

        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(
                dropper::closeClaw,
                dropper::openClaw
        );

        HangSpecimenCommandGroup hangSpecimen = new HangSpecimenCommandGroup(dropper);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(hangSpecimen);


        Trigger intakeSlidesTrigger = new Trigger(() ->
                manipulatorGamepad.getLeftY() != 0
        );
        intakeSlidesTrigger.whileActiveContinuous(() -> intake.moveSlidesRelative(
                manipulatorGamepad.getLeftY() * 2));

        Trigger dropperSlidesTrigger = new Trigger(() ->
                manipulatorGamepad.getRightY() != 0
        );
        dropperSlidesTrigger.whileActiveContinuous(() -> dropper.moveSlidesRelative(
                manipulatorGamepad.getRightY() * 2));

        IntakeManualRotationCommand intakeManualRotationCommand =
                new IntakeManualRotationCommand(intake, manipulatorGamepad);
        Trigger intakeRotationTrigger = new Trigger(() ->
                (manipulatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 ||
                        manipulatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0)
                        && robotState.getIntakeState() != IntakeState.TRANSFER
        );
        intakeRotationTrigger.whileActiveContinuous(intakeManualRotationCommand);

    }
}
