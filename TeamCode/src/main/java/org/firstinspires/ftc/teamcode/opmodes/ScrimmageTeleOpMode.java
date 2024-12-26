package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ChangeBlockColorPreferenceCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.DropperToHighBasketAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.DropperToHighChamberAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.DropperToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.HangSpecimenAction;
import org.firstinspires.ftc.teamcode.commands.IntakeManualRotationCommand;
import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.base.BaseOpMode;

@TeleOp(name = "Scrimmage TeleOp Mode", group = "Scrimmage")
public class ScrimmageTeleOpMode extends BaseOpMode {
    private IntakeSubsystem intake;
    private RobotState robotState;

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        robotState = new RobotState();

        intake = new IntakeSubsystem(hardwareMap, robotState);
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        registerSubsystems(intake, drive, dropper);

        // DRIVER
        ChangeBlockColorPreferenceCommand changeBlockColorPreferenceCommand = new ChangeBlockColorPreferenceCommand(robotState);
        ManualDriveCommand manualDriveCommand = new ManualDriveCommand(drive, driverGamepad);
        drive.setDefaultCommand(manualDriveCommand);

        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(drive::toggleDriveGears);
        
        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(changeBlockColorPreferenceCommand);

        // MANIPULATOR

        // Intake
//        IntakeToPickupAction intakeToPickup =
//                new IntakeToPickupAction(intake, robotState);
//        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(intakeToPickup);
//
//        IntakeToTransferAction intakeToTransfer =
//                new IntakeToTransferAction(intake, robotState);
//        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(intakeToTransfer);

        // Reset the intake slide encoders
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                intake::resetSlides
        );

        // TODO: Removed reference to old IntakeState; need to come back and update
//        Trigger intakeInTransfer = new Trigger(() ->
////                robotState.getIntakeState() == IntakeState.TRANSFER
//        );

//        // Toggles the rotation on the intake between two perpendicular positions
//        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.X).and(intakeInTransfer.negate()).whenActive(
//                intake::togglePerpendicularRotation
//        );

        // Toggles the intake claw between open and closed positions
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intake::toggleClaw);

        Trigger intakeSlidesTrigger = new Trigger(() ->
                manipulatorGamepad.getLeftY() != 0
        );
        intakeSlidesTrigger.whileActiveContinuous(() -> intake.moveSlidesRelative(
                manipulatorGamepad.getLeftY() * 2));

        IntakeManualRotationCommand intakeManualRotationCommand =
                new IntakeManualRotationCommand(intake, manipulatorGamepad);
        Trigger intakeRotationTrigger = new Trigger(() ->
                manipulatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 ||
                        manipulatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0
        );
//        intakeRotationTrigger.and(intakeInTransfer.negate()).whileActiveContinuous(intakeManualRotationCommand);

        // Dropper
        DropperToTransferAction dropperToTransfer =
                new DropperToTransferAction(dropper, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(dropperToTransfer);

        DropperToHighBasketAction highBasketDrop =
                new DropperToHighBasketAction(dropper, intake, robotState);
//        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(intakeInTransfer).whenActive(highBasketDrop);

        Trigger intakeFromWall = new Trigger(() ->
                robotState.getDropperState() == DropperState.WALL_INTAKE
        );

        DropperToHighChamberAction highChamberDropFromIntake =
                new DropperToHighChamberAction(dropper, intake, robotState);

         //If the dropper is transferring a specimen from the intake, activate the high chamber drop from intake command
//        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).and(intakeInTransfer).and(intakeFromWall.negate()).whenActive(highChamberDropFromIntake);

        // Reset the dropper slide encoders
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                dropper::resetSlides
        );

        HangSpecimenAction hangSpecimen = new HangSpecimenAction(dropper);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(hangSpecimen);

        // Toggles the dropper claw between open and closed positions
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                dropper::toggleClaw
        );

        Trigger dropperSlidesTrigger = new Trigger(() ->
                manipulatorGamepad.getRightY() != 0
//                true
        );
        dropperSlidesTrigger.whileActiveContinuous(() ->
                dropper.moveSlidesRelative(
                -manipulatorGamepad.getRightY() * 2.5)
        );
    }

    @Override
    public void update() {
        telemetry.addData("Intake Claw Pos", intake.getClawPosition());
        telemetry.addData("Intake slide pos", intake.getCurrentSlidePositionInches());
    }
}
