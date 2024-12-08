package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.DropperToHighBasketAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.DropperToHighChamberTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.DropperToHighChamberWallAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.DropperToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.DropperToWallAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.HangSpecimenAction;
import org.firstinspires.ftc.teamcode.commands.IntakeManualRotationCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeToTransferAction;
import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
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
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        registerSubsystems(intake, dropper, drive);

        // DRIVER
        ManualDriveCommand manualDriveCommand = new ManualDriveCommand(drive, driverGamepad);
        drive.setDefaultCommand(manualDriveCommand);

        // MANIPULATOR

        // Intake
        IntakeToPickupAction intakeToPickup =
                new IntakeToPickupAction(intake, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(intakeToPickup);

        IntakeToTransferAction intakeToTransfer =
                new IntakeToTransferAction(intake, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(intakeToTransfer);

        // Reset the intake slide encoders
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                intake::resetSlides
        );

        Trigger intakeInTransfer = new Trigger(() ->
                robotState.getIntakeState() == IntakeState.TRANSFER
        );

        // Toggles the rotation on the intake between two perpendicular positions
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.X).and(intakeInTransfer.negate()).toggleWhenActive(
                () -> intake.setWristAbsolute(180, 90),
                () -> intake.setWristAbsolute(180, 0)
        );

        // Toggles the intake claw between open and closed positions
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intake::toggleClaw);

        Trigger intakeSlidesTrigger = new Trigger(() ->
                manipulatorGamepad.getLeftY() != 0
        );
        intakeSlidesTrigger.whileActiveContinuous(() -> intake.moveSlidesRelative(
                manipulatorGamepad.getLeftY()));

        IntakeManualRotationCommand intakeManualRotationCommand =
                new IntakeManualRotationCommand(intake, manipulatorGamepad);
        Trigger intakeRotationTrigger = new Trigger(() ->
                manipulatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 ||
                        manipulatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0
        );
        intakeRotationTrigger.and(intakeInTransfer.negate()).whileActiveContinuous(intakeManualRotationCommand);

        // Dropper
        DropperToWallAction dropperToWall =
                new DropperToWallAction(dropper, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(dropperToWall);

        DropperToTransferAction dropperToTransfer =
                new DropperToTransferAction(dropper, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(dropperToTransfer);

        DropperToHighBasketAction highBasketDrop =
                new DropperToHighBasketAction(dropper, intake, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(intakeInTransfer).whenActive(highBasketDrop);

        Trigger intakeFromWall = new Trigger(() ->
                robotState.getDropperState() == DropperState.WALL_INTAKE
        );

        DropperToHighChamberTransferAction highChamberDropFromIntake =
                new DropperToHighChamberTransferAction(dropper, intake, robotState);
        DropperToHighChamberWallAction highChamberDropFromWall =
                new DropperToHighChamberWallAction(dropper, robotState);

        // If the dropper is intaking a specimen from the wall, activate the high chamber drop from wall command
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).and(intakeFromWall).whenActive(highChamberDropFromWall);

        // If the dropper is transferring a specimen from the intake, activate the high chamber drop from intake command
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).and(intakeInTransfer).and(intakeFromWall.negate()).whenActive(highChamberDropFromIntake);

        // Reset the dropper slide encoders
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                dropper::resetSlides
        );

        HangSpecimenAction hangSpecimen = new HangSpecimenAction(dropper);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(hangSpecimen);

        // Toggles the dropper claw between open and closed positions
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(
                dropper::closeClaw,
                dropper::openClaw
        );

        Trigger dropperSlidesTrigger = new Trigger(() ->
                manipulatorGamepad.getRightY() != 0
        );
        dropperSlidesTrigger.whileActiveContinuous(() -> dropper.moveSlidesRelative(
                manipulatorGamepad.getRightY() * -2));
    }
}
