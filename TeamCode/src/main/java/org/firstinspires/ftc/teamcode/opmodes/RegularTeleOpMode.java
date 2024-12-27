package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ChangeBlockColorPreferenceCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeManualRotationCommand;
import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackwardCarryAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackwardCarryNTAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryNTAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperFrontSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketNTAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeToReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.BaseOpMode;

@TeleOp()
public class RegularTeleOpMode extends BaseOpMode {

    private RobotState robotState;
    private IntakeSubsystem intake;

    @Override
    public void initialize() {
//        FtcDashboard.getInstance();

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

        //MANIPULATOR

        // Intake

        //Intake State Movements
        IntakeTuckAction intakeTuckCommand = new IntakeTuckAction(intake, robotState);
        IntakePrepareToPickupAction intakePrepareToPickupAction = new IntakePrepareToPickupAction(intake, robotState);
        IntakeReadyToPickupAction intakeReadyToPickupAction = new IntakeReadyToPickupAction(intake, robotState);
        IntakePrepareToTransferAction intakePrepareToTransferAction = new IntakePrepareToTransferAction(intake, robotState);
        IntakeToReadyToTransferAction intakeToReadyToTransferAction = new IntakeToReadyToTransferAction(intake, robotState);

        Trigger rightBumper = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        Trigger leftBumper = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);

        Trigger inTuck = new Trigger(() -> robotState.getIntakeState() == IntakeState.TUCK);
        Trigger inPrepareToIntake = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_PICKUP);
        Trigger inReadyToIntake = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_PICKUP);
        Trigger inPrepareToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER);
        Trigger inReadyToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER);

        leftBumper.and(inPrepareToIntake).whenActive(intakeTuckCommand);
        leftBumper.and(inReadyToIntake).whenActive(intakePrepareToPickupAction);
        leftBumper.and(inPrepareToTransfer).whenActive(intakePrepareToPickupAction);
        leftBumper.and(inReadyToTransfer).whenActive(intakePrepareToPickupAction);

        rightBumper.and(inTuck).whenActive(intakePrepareToPickupAction);
        rightBumper.and(inPrepareToIntake).whenActive(intakeReadyToPickupAction);
        rightBumper.and(inReadyToIntake).whenActive(intakePrepareToTransferAction);
        rightBumper.and(inPrepareToTransfer).whenActive(intakeToReadyToTransferAction);

        //Other Intake Stuff
        // Reset the intake slide encoders
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                intake::resetSlides
        );

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
                (manipulatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 ||
                        manipulatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0)
                        && robotState.getIntakeState() == IntakeState.READY_TO_PICKUP
        );
        intakeRotationTrigger.whileActiveContinuous(intakeManualRotationCommand);

        //Dropper

        //Dropper State Transitions
        DropperBackSlapAction dropperBackSlapAction = new DropperBackSlapAction(dropper, robotState);
        DropperFrontSlapAction dropperFrontSlapAction = new DropperFrontSlapAction(dropper, robotState);
        DropperBackwardCarryNTAction dropperBackwardCarryNTAction = new DropperBackwardCarryNTAction(dropper, robotState);
        DropperBackwardCarryAction dropperBackwardCarryAction = new DropperBackwardCarryAction(dropper, intake, robotState);
        DropperForwardCarryNTAction dropperForwardCarryNTAction = new DropperForwardCarryNTAction(dropper, robotState);
        DropperForwardCarryAction dropperForwardCarryAction = new DropperForwardCarryAction(dropper, intake, robotState);
        DropperHighBasketNTAction dropperHighBasketNTAction = new DropperHighBasketNTAction(dropper, robotState);
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
        dpadUp.and(blockInIntake.negate()).whenActive(dropperHighBasketNTAction);

        dpadLeft.and(backwardCarry).whenActive(dropperBackSlapAction);
        dpadLeft.and(backwardCarry.negate()).and(blockInIntake).whenActive(dropperBackwardCarryAction);
        dpadLeft.and(backwardCarry.negate()).and(blockInIntake.negate()).whenActive(dropperBackwardCarryNTAction);

        dpadRight.and(forwardCarry).whenActive(dropperFrontSlapAction);
        dpadRight.and(forwardCarry.negate()).and(blockInIntake).whenActive(dropperForwardCarryAction);
        dpadRight.and(forwardCarry.negate()).and(blockInIntake.negate()).whenActive(dropperForwardCarryNTAction);

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
    }
}
