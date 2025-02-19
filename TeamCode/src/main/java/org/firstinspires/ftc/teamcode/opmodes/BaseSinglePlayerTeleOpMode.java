package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryWallAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperFrontSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFullReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFullReadyToTransferNoVisionAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeToObservationZoneAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.SmallCameraVisionPickup;
import org.firstinspires.ftc.teamcode.commands.actions.drive.CancelDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
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
public abstract class BaseSinglePlayerTeleOpMode extends BaseOpMode {
    private RobotState robotState;
    private IntakeSubsystem intake;
    private DropperSubsystem dropper;

    protected abstract boolean isBlue();

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        GamepadEx playerGamepad = new GamepadEx(gamepad1);
        robotState = new RobotState(isBlue(), false);
        robotState.setBlockColorPreference(BlockColorPreference.ANY);
        intake = new IntakeSubsystem(hardwareMap, robotState);
        dropper = new DropperSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        AscentSubsystem ascent = new AscentSubsystem(hardwareMap, robotState);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry;
        try {
            odometry = new GoBodometrySubsystem(hardwareMap, robotState, (Waypoint) RobotSaveState.getInstance().getState("robotCurrentPose"));
        } catch (Exception e) {
            odometry = new GoBodometrySubsystem(hardwareMap, robotState);
        }

        registerSubsystems(intake, drive, dropper, limelight,
                odometry, ascent, sensor);

        // ASCENT
        Trigger startAscentTrigger =
                new Trigger(() -> gamepad1.touchpad_finger_2 || gamepad1.guide);
        Trigger isAscending = new Trigger(() -> robotState.getIsAscending());

        ManualAscentCommand manualAscentCommand = new ManualAscentCommand(robotState,
                () -> -playerGamepad.getRightY(), ascent, dropper, drive);
        StartAscentAction startAscentAction = new StartAscentAction(robotState, ascent, dropper);

        startAscentTrigger.whenActive(startAscentAction);

        Trigger runningEngageAscent =
                new Trigger(() -> CommandScheduler.getInstance().isScheduled(startAscentAction));
        isAscending.and(runningEngageAscent.negate()).whileActiveOnce(manualAscentCommand);

        // DRIVER TODO: Split into a different method
        ManualDriveCommand manualDriveCommand = new ManualDriveCommand(drive,
                playerGamepad);
        drive.setDefaultCommand(manualDriveCommand);

        // MANIPULATOR

        // Intake TODO: Split into a different method

        // Commands
        IntakeToObservationZoneAction intakeToObservation =
                new IntakeToObservationZoneAction(intake, dropper, robotState);
        IntakeTuckAction tuck = new IntakeTuckAction(intake, robotState);
        IntakePrepareToPickupAction prepareToPickupManual = new IntakePrepareToPickupAction(
                intake, dropper, robotState, 5);
        IntakePrepareToPickupAction prepareToPickupAuto = new IntakePrepareToPickupAction(
                intake, dropper, () -> robotState.getBlockForwardCoarse(), robotState);
        IntakePrepareToPickupAction prepareToPickupNoSlides = new IntakePrepareToPickupAction(
                intake, dropper, () -> intake.getCurrentSlidePositionInches(), robotState);
        IntakeReadyToPickupAction readyToPickupManual = new IntakeReadyToPickupAction(
                intake, robotState, IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION);
        SmallCameraVisionPickup readyToPickupAuto = new SmallCameraVisionPickup(intake, dropper,robotState, null
        );
        IntakePrepareToTransferAction prepareToTransferAction = new IntakePrepareToTransferAction(intake, dropper, robotState);
        IntakeFullReadyToTransferAction fullReadyToTransfer = new IntakeFullReadyToTransferAction(
                drive, intake, dropper, robotState);
        IntakeFullReadyToTransferNoVisionAction fullReadyToTransferNoVision = new IntakeFullReadyToTransferNoVisionAction(
                intake, dropper, robotState);
        IntakeVisionPickupAction fullReadyToPickupAuto = new IntakeVisionPickupAction(
                intake, dropper, drive, robotState, () -> robotState.getRobotCurrentPose().getHeading());

        // Button Triggers + Manual trigger
        Trigger rightBumper = playerGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        Trigger leftBumper = playerGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        Trigger isManual = new Trigger(() -> robotState.isManualIntakeSelected());

        Trigger autoExtendTrigger = rightBumper.and(isManual.negate());
        Trigger manualExtendTrigger = rightBumper.and(isManual);
        Trigger autoRetractTrigger = leftBumper.and(isManual.negate());
        Trigger manualRetractTrigger = leftBumper.and(isManual);

        // State triggers
        Trigger inTuck = new Trigger(() -> robotState.getIntakeState() == IntakeState.TUCK);
        Trigger inPrepareToPickup = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_PICKUP);
        Trigger inReadyToPickup = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_PICKUP);
        Trigger inPrepareToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER);
        Trigger inReadyToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER);
        Trigger blockDetected = new Trigger(() -> robotState.getCoarseBlockDetectionState() == BlockDetectionState.DETECTED);

        // Retract Trigger bindings
        (manualRetractTrigger.or(autoRetractTrigger)).and(inReadyToTransfer).whenActive(prepareToPickupManual);

        (manualRetractTrigger.or(autoRetractTrigger)).and(inPrepareToPickup.or(inTuck)).whenActive(tuck);
        (manualRetractTrigger.or(autoRetractTrigger)).and(inReadyToPickup).whenActive(prepareToPickupNoSlides);

        // Uses the full vision pickup if the block is detected, runs the manual one if not
//        autoRetractTrigger.and(inReadyToTransfer).and(blockDetected).whenActive(fullReadyToPickupAuto);
//        autoRetractTrigger.and(inReadyToTransfer).and(blockDetected.negate()).whenActive(
//                () -> {
//                    prepareToPickupManual.schedule();
//                    gamepad2.rumbleBlips(3);
//                }
//        );

        // Extend Trigger Bindings
        (manualExtendTrigger.or(autoExtendTrigger)).and(inTuck).whenActive(prepareToPickupManual);
        (manualExtendTrigger.or(autoExtendTrigger)).and(inPrepareToPickup).whenActive(readyToPickupManual);
        autoExtendTrigger.and(inReadyToPickup).whenActive(fullReadyToTransfer);
        manualExtendTrigger.and(inReadyToPickup).whenActive(fullReadyToTransferNoVision);

//        manualExtendTrigger.or(autoExtendTrigger).and(inReadyToPickup).whenActive(fullReadyToTransfer);
        manualExtendTrigger.or(autoExtendTrigger).and(inReadyToTransfer).whenActive(intakeToObservation);

        // Uses the full vision pickup if the block is detected, runs the manual one if not
//        autoExtendTrigger.and(inTuck).and(blockDetected).whenActive(fullReadyToPickupAuto);
//        autoExtendTrigger.and(inTuck).and(blockDetected.negate()).whenActive(
//                () -> {
//                    prepareToPickupManual.schedule();
//                    gamepad2.rumbleBlips(3);
//                }
//        );

        // TODO: Make auto later when small cam works
//        autoExtendTrigger.and(inPrepareToPickup).whenActive(readyToPickupAuto);

        // Other Intake Stuff

        // Toggles manual intake mode
        playerGamepad.getGamepadButton(GamepadKeys.Button.START).toggleWhenPressed(
                () -> {
                    robotState.setManualIntakeSelected(true);
                    gamepad1.rumbleBlips(1);
                },
                () -> {
                    robotState.setManualIntakeSelected(false);
                    gamepad1.rumbleBlips(2);
                }
        );

        // Toggles the intake claw between open and closed positions
        playerGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intake::toggleClaw);

        // Controls intake slides
        Trigger intakeSlidesTrigger = new Trigger(() ->
                gamepad1.right_trigger - gamepad1.left_trigger != 0
        );

        intakeSlidesTrigger.whileActiveContinuous(() -> intake.moveSlidesRelative(
                (gamepad1.right_trigger - gamepad1.left_trigger) * 1.7));

        // Intake claw rotation toggle to 0 or 90
        playerGamepad.getGamepadButton(GamepadKeys.Button.X).and(inReadyToPickup)
                .whenActive(intake::togglePerpendicularRotation);

        // Changing Color Preference
        ChangeBlockColorPreferenceCommand changeBlockColorPreferenceCommand =
                new ChangeBlockColorPreferenceCommand(robotState, playerGamepad);
        playerGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                changeBlockColorPreferenceCommand);

        // Dropper TODO: Split into a different method

        // Dropper State Transitions
        DropperForwardCarryWallAction dropperForwardCarryWallAction = new DropperForwardCarryWallAction(dropper, robotState);
        DropperWallIntakeAction dropperWallIntakeAction =
                new DropperWallIntakeAction(dropper, intake, robotState);
        DropperWallIntakeNoTransferAction dropperWallIntakeNoTransferAction = new DropperWallIntakeNoTransferAction(dropper, robotState);
        DropperBackSlapAction dropperBackSlapAction = new DropperBackSlapAction(dropper, robotState);
        DropperFrontSlapAction dropperFrontSlapAction = new DropperFrontSlapAction(dropper, robotState);
        DropperBackwardCarryNoTransferAction dropperBackwardCarryNoTransferAction = new DropperBackwardCarryNoTransferAction(dropper, robotState);
        DropperBackwardCarryAction dropperBackwardCarryAction = new DropperBackwardCarryAction(dropper, intake, robotState);
        DropperForwardCarryNoTransferAction dropperForwardCarryNoTransferAction = new DropperForwardCarryNoTransferAction(dropper, robotState);
        DropperForwardCarryAction dropperForwardCarryAction = new DropperForwardCarryAction(dropper, intake, robotState);
        DropperHighBasketNoTransferAction dropperHighBasketNoTransferAction = new DropperHighBasketNoTransferAction(dropper, robotState);
        DropperHighBasketAction dropperHighBasketAction = new DropperHighBasketAction(dropper, intake, robotState);
        DropperPreTransferAction dropperPreTransferAction = new DropperPreTransferAction(dropper, robotState);
        DropperTransferAction dropperTransferAction = new DropperTransferAction(dropper, robotState);

        Trigger dpadLeft =
                playerGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        Trigger dpadRight =
                playerGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        Trigger dpadLeftAndRight =
                playerGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).and(playerGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT));
        Trigger back =
                playerGamepad.getGamepadButton(GamepadKeys.Button.BACK);
        Trigger dpadUp = playerGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        Trigger dpadDown = playerGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);

        Trigger forwardCarry = new Trigger(() -> robotState.getDropperState() == DropperState.FORWARD_CARRY);
        Trigger backwardCarry = new Trigger(() -> robotState.getDropperState() == DropperState.BACKWARD_CARRY);
        Trigger transfer =
                new Trigger(() -> robotState.getDropperState() == DropperState.PRE_TRANSFER || robotState.getDropperState() == DropperState.TRANSFER);
        Trigger wallIntake = new Trigger(() -> robotState.getDropperState() == DropperState.WALL_INTAKE);
        Trigger blockInDropper = new Trigger(() -> robotState.getBlockPosition() == RobotBlockPosition.DROPPER);
        Trigger blockInIntake = new Trigger(() -> robotState.getBlockPosition() == RobotBlockPosition.INTAKE);

        // Back down
        dpadDown.and(blockInIntake).whenActive(dropperTransferAction);
        dpadDown.and(blockInIntake.negate()).whenActive(dropperPreTransferAction);

        // Basket drop
        dpadUp.and(blockInIntake).whenActive(dropperHighBasketAction);
        dpadUp.and(blockInIntake.negate()).whenActive(dropperHighBasketNoTransferAction);

        // Wall Intake
        dpadLeft.and(wallIntake.negate()).and(blockInIntake).whenActive(dropperWallIntakeAction);
        dpadLeft.and(wallIntake.negate()).and(blockInIntake.negate()).whenActive(dropperWallIntakeNoTransferAction);
        dpadRight.and(wallIntake).whenActive(dropperForwardCarryWallAction);

        // Dropper specimen movements
        back.and(backwardCarry).whenActive(dropperBackSlapAction);
        back.and(blockInIntake).and(transfer).whenActive(dropperBackwardCarryAction);
        back.and(backwardCarry.negate()).and(blockInDropper).whenActive(dropperBackwardCarryNoTransferAction);

        dpadRight.and(forwardCarry).whenActive(dropperFrontSlapAction);
        dpadRight.and(blockInIntake).and(transfer).whenActive(dropperForwardCarryAction);
        dpadRight.and(forwardCarry.negate()).and(blockInDropper).whenActive(dropperForwardCarryNoTransferAction);

        //Manual Dropper Stuff

        // Toggles the dropper claw between open and closed positions
        playerGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                dropper::toggleClaw
        );

        Trigger unsafeDropper = playerGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        UnsafeDropperSlidesCommand unsafeDropperSlidesCommand = new UnsafeDropperSlidesCommand(dropper, playerGamepad);

        Trigger dropperSlidesTrigger = new Trigger(() ->
                playerGamepad.getRightY() != 0
        );
        dropperSlidesTrigger.and(unsafeDropper.negate()).and(isAscending.negate()).whileActiveContinuous(() ->
                dropper.moveSlidesRelative(
                        -playerGamepad.getRightY() * 2.5)
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
        }));

        Trigger finalRumble = new Trigger(() -> robotState.getRunTime() > 115000);

        endgameRumbleTrigger.whileActiveOnce(new InstantCommand(() -> {
            gamepad1.rumbleBlips(5);
        }));

        telemetry.addData("Voltage: ", robotState.getVoltage());
        telemetry.update();
    }

    @Override
    public void justAfterStart() {
        robotState.resetTimer();
    }

    @Override
    public void update() {
        telemetry.addData("Intake State", robotState.getIntakeState());
        telemetry.addData("Dropper State", robotState.getDropperState());
        telemetry.addData("Intake Slide POS",
                intake.getCurrentSlidePositionInches());
        telemetry.addData("Dropper Slide POS",
                dropper.getCurrentSlidePositionInches());
        telemetry.addData("Manual Intake?", robotState.isManualIntakeSelected());
        telemetry.addData("Block Detection State", robotState.getCoarseBlockDetectionState());
        telemetry.addData("Current Block Preference", robotState.getBlockColorPreference());
//        telemetry.addData("Robot pose", robotState.getRobotCurrentPose());
        telemetry.addData("vision intake heading", Math.toDegrees(robotState.getVisionIntakeHeading()));
        telemetry.addLine();
        telemetry.addData("Velocity: ", robotState.getRobotVelocity().getPoint().magnitude());
        telemetry.addData("Heading Velocity: ", Math.toDegrees(robotState.getRobotVelocity().getHeading()));
        telemetry.addLine();
        telemetry.addData("Runtime: ", robotState.getRunTime());
        telemetry.addData("Voltage: ", robotState.getVoltage());
        telemetry.addData("Block Color: ", robotState.getIntakeBlockColor());
        telemetry.addLine();
        telemetry.addData("Robot X: ", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Robot Y: ", robotState.getRobotCurrentPose().getY());
        telemetry.addData("Lateral Distance from Block", robotState.getBlockLateralFine());
        telemetry.addData("Forward Distance from Block", robotState.getBlockForwardFine());
        telemetry.addData("Block Orientation", robotState.getBlockOrientation());
//        telemetry.addData("Intake Claw Distance from Block", robotState.getBlockForwardCoarse());
        telemetry.addLine();
        telemetry.addData("Break Beam Sensor", robotState.getBlockPosition());
    }
}
