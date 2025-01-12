package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakeManualRotationCommand;
import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackwardCarryAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackwardCarryNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperFrontSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.BaseOpMode;

@TeleOp
@SuppressWarnings("unused")
public class RegularTeleOpMode extends BaseOpMode {
    private static final double INTAKE_CAMERA_OFFSET = 2;
    private RobotState robotState;
    private IntakeSubsystem intake;

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        robotState = new RobotState(false, false);
        robotState.setBlockColorPreference(BlockColorPreference.ANY);

        intake = new IntakeSubsystem(hardwareMap, robotState);
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        VisionSubsystem smallCamera = new VisionSubsystem(hardwareMap, robotState);
        registerSubsystems(intake, drive, dropper, smallCamera);

        // DRIVER TODO: Split into a different method
        ManualDriveCommand manualDriveCommand = new ManualDriveCommand(drive, driverGamepad);
        drive.setDefaultCommand(manualDriveCommand);

        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(drive::toggleDriveGears);

        // MANIPULATOR

        // Intake TODO: Split into a different method

        // Commands
        IntakeTuckAction tuck = new IntakeTuckAction(intake, robotState);
        IntakePrepareToPickupAction prepareToPickupManual = new IntakePrepareToPickupAction(
                intake, dropper, robotState, 10);
        IntakePrepareToPickupAction prepareToPickupAuto = new IntakePrepareToPickupAction(
                intake, dropper, robotState, () -> robotState.getBlockForwardCoarse());
        IntakePrepareToPickupAction prepareToPickupNoSlides = new IntakePrepareToPickupAction(
                intake, dropper, robotState, () -> intake.getCurrentSlidePositionInches());
        IntakeReadyToPickupAction readyToPickupManual = new IntakeReadyToPickupAction(
                intake, robotState, IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION);
        IntakeReadyToPickupAction readyToPickupAuto = new IntakeReadyToPickupAction(intake, robotState,
                () -> intake.getCurrentSlidePositionInches() + robotState.getBlockForwardFine()-INTAKE_CAMERA_OFFSET,
                () -> (robotState.getBlockOrientation() + 180) % 180); // This is done to translate claw rotation to block orientation
        IntakePrepareToTransferAction prepareToTransfer = new IntakePrepareToTransferAction(
                intake, dropper, robotState);
        IntakeReadyToTransferAction readyToTransfer = new IntakeReadyToTransferAction(
                intake, robotState);

        // Button Triggers + Manual trigger
        Trigger rightBumper = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        Trigger leftBumper = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        Trigger isManual = new Trigger(() -> robotState.isManualIntakeSelected());

        Trigger autoExtendTrigger = rightBumper.and(isManual.negate());
        Trigger manualExtendTrigger = rightBumper.and(isManual);
        Trigger autoRetractTrigger = leftBumper.and(isManual.negate());
        Trigger manualRetractTrigger = leftBumper.and(isManual);

        // State triggers
        Trigger inTuck = new Trigger(() -> robotState.getIntakeState() == IntakeState.TUCK);
        Trigger inPrepareToIntake = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_PICKUP);
        Trigger inReadyToIntake = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_PICKUP);
        Trigger inPrepareToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER);
        Trigger inReadyToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER);

        // Retract Trigger bindings
        manualRetractTrigger.and(inReadyToTransfer).whenActive(prepareToPickupManual);
        manualRetractTrigger.and(inPrepareToTransfer).whenActive(prepareToPickupManual);

        manualRetractTrigger.or(autoRetractTrigger).and(inPrepareToIntake).whenActive(tuck);
        manualRetractTrigger.or(autoRetractTrigger).and(inReadyToIntake).whenActive(prepareToPickupNoSlides);

        // TODO: Change to Prepare to pickup auto for both
        autoRetractTrigger.and(inReadyToTransfer).whenActive(prepareToPickupManual);
        autoRetractTrigger.and(inPrepareToTransfer).whenActive(prepareToPickupManual);

        // Extend Trigger Bindings
        manualExtendTrigger.and(inTuck).whenActive(prepareToPickupManual);
        manualExtendTrigger.and(inPrepareToIntake).whenActive(readyToPickupManual);

        manualExtendTrigger.or(autoExtendTrigger).and(inReadyToIntake).whenActive(prepareToTransfer);
        manualExtendTrigger.or(autoExtendTrigger).and(inPrepareToTransfer).whenActive(readyToTransfer);

        // TODO: Change to Prepare to pickup auto
        autoExtendTrigger.and(inTuck).whenActive(prepareToPickupManual);
        autoExtendTrigger.and(inPrepareToIntake).whenActive(readyToPickupAuto);

        // Other Intake Stuff

        // Toggles manual intake mode
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.START).toggleWhenPressed(
                () -> robotState.setManualIntakeSelected(true),
                () -> robotState.setManualIntakeSelected(false)
        );

        // Reset the intake slide encoders
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                intake::resetSlides
        );

        // Toggles the intake claw between open and closed positions
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intake::toggleClaw);

        // Controls intake slides
        Trigger intakeSlidesTrigger = new Trigger(() ->
                manipulatorGamepad.getLeftY() != 0
        );
        intakeSlidesTrigger.whileActiveContinuous(() -> intake.moveSlidesRelative(
                manipulatorGamepad.getLeftY() * 2));

        // Manual intake rotation
        IntakeManualRotationCommand intakeManualRotationCommand =
                new IntakeManualRotationCommand(intake, manipulatorGamepad);
        Trigger intakeRotationTrigger = new Trigger(() ->
                (manipulatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 ||
                        manipulatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0)
        );
        intakeRotationTrigger.and(inReadyToIntake).whileActiveContinuous(intakeManualRotationCommand);

        // Intake claw rotation toggle to 0 or 90
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.X).and(inReadyToIntake)
                .whenActive(intake::togglePerpendicularRotation);

        // Dropper TODO: Split into a different method

        // Dropper State Transitions
        DropperBackSlapAction dropperBackSlapAction = new DropperBackSlapAction(dropper, robotState);
        DropperFrontSlapAction dropperFrontSlapAction = new DropperFrontSlapAction(dropper, robotState);
        DropperBackwardCarryNoTransferAction dropperBackwardCarryNoTransferAction = new DropperBackwardCarryNoTransferAction(dropper, robotState);
        DropperBackwardCarryAction dropperBackwardCarryAction = new DropperBackwardCarryAction(dropper, intake, robotState);
        DropperForwardCarryNoTransferAction dropperForwardCarryNoTransferAction = new DropperForwardCarryNoTransferAction(dropper, robotState);
        DropperForwardCarryAction dropperForwardCarryAction = new DropperForwardCarryAction(dropper, intake, robotState);
        DropperHighBasketNoTransferAction dropperHighBasketNoTransferAction = new DropperHighBasketNoTransferAction(dropper, robotState);
        DropperHighBasketAction dropperHighBasketAction = new DropperHighBasketAction(dropper, intake, robotState);
        DropperPreTransferAction dropperPreTransferAction = new DropperPreTransferAction(dropper, robotState);

        Trigger dpadLeft = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        Trigger dpadRight = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        Trigger dpadUp = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        Trigger dpadDown = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);

        Trigger forwardCarry = new Trigger(() -> robotState.getDropperState() == DropperState.FORWARD_CARRY);
        Trigger backwardCarry = new Trigger(() -> robotState.getDropperState() == DropperState.BACKWARD_CARRY);
        Trigger blockInIntake = new Trigger(() -> robotState.getBlockPosition() == RobotBlockPosition.INTAKE);

        dpadDown.whenActive(dropperPreTransferAction);

        dpadUp.and(blockInIntake).whenActive(dropperHighBasketAction);
        dpadUp.and(blockInIntake.negate()).whenActive(dropperHighBasketNoTransferAction);

        dpadLeft.and(backwardCarry).whenActive(dropperBackSlapAction);
        dpadLeft.and(backwardCarry.negate()).and(blockInIntake).whenActive(dropperBackwardCarryAction);
        dpadLeft.and(backwardCarry.negate()).and(blockInIntake.negate()).whenActive(dropperBackwardCarryNoTransferAction);

        dpadRight.and(forwardCarry).whenActive(dropperFrontSlapAction);
        dpadRight.and(forwardCarry.negate()).and(blockInIntake).whenActive(dropperForwardCarryAction);
        dpadRight.and(forwardCarry.negate()).and(blockInIntake.negate()).whenActive(dropperForwardCarryNoTransferAction);

        //Manual Dropper Stuff
        // Reset the dropper slide encoders
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                dropper::resetSlides
        );

        // Toggles the dropper claw between open and closed positions
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                dropper::toggleClaw
        );

        Trigger dropperSlidesTrigger = new Trigger(() ->
                manipulatorGamepad.getRightY() != 0
        );
        dropperSlidesTrigger.whileActiveContinuous(() ->
                dropper.moveSlidesRelative(
                        -manipulatorGamepad.getRightY() * 2.5)
        );
    }

    @Override
    public void update() {
        telemetry.addData("Intake State", robotState.getIntakeState());
        telemetry.addData("Dropper State", robotState.getDropperState());
        telemetry.addData("Slide POS", intake.getCurrentSlidePositionInches());
        telemetry.addData("Manual Intake?", robotState.isManualIntakeSelected());
    }
}
