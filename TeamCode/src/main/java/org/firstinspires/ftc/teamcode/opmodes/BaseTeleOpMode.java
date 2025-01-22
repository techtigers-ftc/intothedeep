package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.ChangeBlockColorPreferenceCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeManualRotationCommand;
import org.firstinspires.ftc.teamcode.commands.ManualAscentCommand;
import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.commands.UnsafeDropperSlidesCommand;
import org.firstinspires.ftc.teamcode.commands.UnsafeIntakeSlidesCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeVisionPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.ascent.StartAscentAction;
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
import org.firstinspires.ftc.teamcode.commands.actions.drive.CancelDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.BaseOpMode;
import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.utils.RobotSaveState;

@SuppressWarnings("unused")
public abstract class BaseTeleOpMode extends BaseOpMode {
    private RobotState robotState;
    private IntakeSubsystem intake;

    protected abstract boolean isBlue();

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        robotState = new RobotState(isBlue(), false);
        robotState.setBlockColorPreference(BlockColorPreference.ANY);

        intake = new IntakeSubsystem(hardwareMap, robotState);
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        AscentSubsystem ascent = new AscentSubsystem(hardwareMap, robotState);
        VisionSubsystem smallCamera = new VisionSubsystem(hardwareMap, robotState);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState, 10.5, 2.9, 6, 25);
        GoBodometrySubsystem odometry;
        try {
            odometry = new GoBodometrySubsystem(hardwareMap, robotState, (Waypoint) RobotSaveState.getInstance().getState("robotCurrentPose"));
        } catch (Exception e) {
            odometry = new GoBodometrySubsystem(hardwareMap, robotState);
        }
        registerSubsystems(intake, drive, dropper, smallCamera, limelight,
                odometry, ascent);

        // ASCENT
        Trigger startAscentTrigger =
                new Trigger(() -> gamepad1.touchpad_finger_2);
        Trigger isAscending = new Trigger(() -> robotState.getIsAscending());

        ManualAscentCommand manualAscentCommand = new ManualAscentCommand(robotState,
                () -> -manipulatorGamepad.getRightY(), ascent, dropper, drive);
        StartAscentAction startAscentAction = new StartAscentAction(robotState, ascent, dropper);

        startAscentTrigger.whenActive(startAscentAction);

        Trigger runningEngageAscent =
                new Trigger(() -> CommandScheduler.getInstance().isScheduled(startAscentAction));
        isAscending.and(runningEngageAscent.negate()).whileActiveOnce(manualAscentCommand);

        // DRIVER TODO: Split into a different method
        ManualDriveCommand manualDriveCommand = new ManualDriveCommand(drive, driverGamepad);
        drive.setDefaultCommand(manualDriveCommand);

        CancelDriveCommand cancelDriveCommand = new CancelDriveCommand(drive);
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(cancelDriveCommand);

        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                () -> robotState.setCurrentGear(DriveGears.ENGAGED));
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                () -> robotState.setCurrentGear(DriveGears.NOT_ENGAGED));


        // MANIPULATOR

        // Intake TODO: Split into a different method

        // Commands
        IntakeReadyToPickupAction intakeToObservation =
                new IntakeReadyToPickupAction(intake, robotState,
                        () -> 5, () -> 90);
        IntakeTuckAction tuck = new IntakeTuckAction(intake, robotState);
        IntakePrepareToPickupAction prepareToPickupManual = new IntakePrepareToPickupAction(
                intake, dropper, robotState, 5);
        IntakePrepareToPickupAction prepareToPickupAuto = new IntakePrepareToPickupAction(
                intake, dropper, robotState, () -> robotState.getBlockForwardCoarse());
        IntakePrepareToPickupAction prepareToPickupNoSlides = new IntakePrepareToPickupAction(
                intake, dropper, robotState, () -> intake.getCurrentSlidePositionInches());
        IntakeReadyToPickupAction readyToPickupManual = new IntakeReadyToPickupAction(
                intake, robotState, IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION);
        IntakeReadyToPickupAction readyToPickupAuto = new IntakeReadyToPickupAction(intake, robotState,
                () -> intake.getCurrentSlidePositionInches() + robotState.getBlockForwardFine() - VisionSubsystem.INTAKE_CAMERA_OFFSET,
                () -> (robotState.getBlockOrientation() + 180) % 180); // This is done to translate claw rotation to block orientation
        IntakePrepareToTransferAction prepareToTransfer = new IntakePrepareToTransferAction(
                intake, dropper, robotState);
        IntakeReadyToTransferAction readyToTransfer = new IntakeReadyToTransferAction(
                intake, robotState);
        IntakeVisionPickupAction fullReadyToPickupAuto = new IntakeVisionPickupAction(
                intake, dropper, drive, robotState, robotState::getVisionIntakeHeading, driverGamepad);

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
        Trigger blockDetected = new Trigger(() -> robotState.getBlockDetectionState() == BlockDetectionState.DETECTED);

        // Retract Trigger bindings
        manualRetractTrigger.and(inReadyToTransfer).whenActive(prepareToPickupManual);
        manualRetractTrigger.and(inPrepareToTransfer).whenActive(prepareToPickupManual);

        manualRetractTrigger.or(autoRetractTrigger).and(inPrepareToIntake.or(inTuck)).whenActive(tuck);
        manualRetractTrigger.or(autoRetractTrigger).and(inReadyToIntake).whenActive(prepareToPickupNoSlides);

        // Uses the full vision pickup if the block is detected, runs the manual one if not
        autoRetractTrigger.and(inReadyToTransfer).and(blockDetected).whenActive(fullReadyToPickupAuto);
        autoRetractTrigger.and(inReadyToTransfer).and(blockDetected.negate()).whenActive(
                () -> {
                    prepareToPickupManual.schedule();
                    gamepad2.rumbleBlips(3);
                }
        );

        autoRetractTrigger.and(inPrepareToTransfer).whenActive(prepareToPickupManual);

        // Extend Trigger Bindings
        manualExtendTrigger.and(inTuck).whenActive(prepareToPickupManual);
        manualExtendTrigger.and(inPrepareToIntake).whenActive(readyToPickupManual);

        manualExtendTrigger.or(autoExtendTrigger).and(inReadyToIntake).whenActive(prepareToTransfer);
        manualExtendTrigger.or(autoExtendTrigger).and(inPrepareToTransfer).whenActive(readyToTransfer);
        manualExtendTrigger.or(autoExtendTrigger).and(inReadyToTransfer).whenActive(intakeToObservation);

        // Uses the full vision pickup if the block is detected, runs the manual one if not
        autoExtendTrigger.and(inTuck).and(blockDetected).whenActive(fullReadyToPickupAuto);
        autoExtendTrigger.and(inTuck).and(blockDetected.negate()).whenActive(
                () -> {
                    prepareToPickupManual.schedule();
                    gamepad2.rumbleBlips(3);
                }
        );

        autoExtendTrigger.and(inPrepareToIntake).whenActive(readyToPickupAuto);

        // Other Intake Stuff

        // Toggles manual intake mode
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.START).toggleWhenPressed(
                () -> {
                    robotState.setManualIntakeSelected(true);
                    gamepad2.rumbleBlips(1);
                },
                () -> {
                    robotState.setManualIntakeSelected(false);
                    gamepad2.rumbleBlips(2);
                }
        );

        // Toggles the intake claw between open and closed positions
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intake::toggleClaw);

        // Controls intake slides
        UnsafeIntakeSlidesCommand unsafeIntakeSlidesCommand = new UnsafeIntakeSlidesCommand(intake, manipulatorGamepad);

        Trigger unsafeIntake = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
        Trigger intakeSlidesTrigger = new Trigger(() ->
                manipulatorGamepad.getLeftY() != 0
        );

        intakeSlidesTrigger.and(unsafeIntake.negate()).whileActiveContinuous(() -> intake.moveSlidesRelative(
                manipulatorGamepad.getLeftY() * 2));
        intakeSlidesTrigger.and(unsafeIntake).whileActiveOnce(unsafeIntakeSlidesCommand);

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

        // Changing Color Preference
        ChangeBlockColorPreferenceCommand changeBlockColorPreferenceCommand =
                new ChangeBlockColorPreferenceCommand(robotState, manipulatorGamepad);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                    changeBlockColorPreferenceCommand);

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

        // Toggles the dropper claw between open and closed positions
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                dropper::toggleClaw
        );

        Trigger unsafeDropper = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        UnsafeDropperSlidesCommand unsafeDropperSlidesCommand = new UnsafeDropperSlidesCommand(dropper, manipulatorGamepad);

        Trigger dropperSlidesTrigger = new Trigger(() ->
                manipulatorGamepad.getRightY() != 0
        );
        dropperSlidesTrigger.and(unsafeDropper.negate()).and(isAscending.negate()).whileActiveContinuous(() ->
                dropper.moveSlidesRelative(
                        -manipulatorGamepad.getRightY() * 2.5)
        );

        unsafeDropper.whileActiveOnce(unsafeDropperSlidesCommand);

        // Endgame RUMBLE

        Trigger endgameRumbleTrigger = new Trigger(() -> robotState.getRunTime() > 90000);

        endgameRumbleTrigger.whileActiveOnce(new InstantCommand(() -> {
            Gamepad.RumbleEffect endgameRumbleEffect = new Gamepad.RumbleEffect.Builder()
                    .addStep(
                        1, 1, 1000
                    )
                    .addStep(
                            0, 0, 500
                    )
                    .addStep(
                            1, 1, 1000
                    )
                    .build();
            gamepad1.runRumbleEffect(endgameRumbleEffect);
            gamepad2.runRumbleEffect(endgameRumbleEffect);
        }));

        Trigger finalRumble = new Trigger(() -> robotState.getRunTime() > 115000);

        endgameRumbleTrigger.whileActiveOnce(new InstantCommand(() -> {
            gamepad1.rumbleBlips(5);
            gamepad2.rumbleBlips(5);
        }));
    }

    @Override
    public void justAfterStart() {
        robotState.resetTimer();
    }

    @Override
    public void update() {
        telemetry.addData("Intake State", robotState.getIntakeState());
        telemetry.addData("Dropper State", robotState.getDropperState());
        telemetry.addData("Slide POS", intake.getCurrentSlidePositionInches());
        telemetry.addData("Manual Intake?", robotState.isManualIntakeSelected());
        telemetry.addData("Block Detection State", robotState.getBlockDetectionState());
        telemetry.addData("Current Block Preference", robotState.getBlockColorPreference());
//        telemetry.addData("Robot pose", robotState.getRobotCurrentPose());
        telemetry.addData("vision intake heading", Math.toDegrees(robotState.getVisionIntakeHeading()));
        telemetry.addLine();
        telemetry.addData("Velocity: ", robotState.getRobotVelocity().getPoint().magnitude());
        telemetry.addData("Heading Velocity: ", Math.toDegrees(robotState.getRobotVelocity().getHeading()));
        telemetry.addLine();
        telemetry.addData("Runtime: ", robotState.getRunTime());
    }
}
