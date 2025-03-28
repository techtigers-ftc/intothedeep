package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.AscendOneLevelCommand;
import org.firstinspires.ftc.teamcode.commands.ChangeBlockColorPreferenceCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeManualRotationCommand;
import org.firstinspires.ftc.teamcode.commands.ManualAscentCommand;
import org.firstinspires.ftc.teamcode.commands.StartAscentCommandGroup;
import org.firstinspires.ftc.teamcode.commands.UnsafeDropperSlidesCommand;
import org.firstinspires.ftc.teamcode.commands.UnsafeIntakeSlidesCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.AutoSpecimenCycleAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackwardCarryAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackwardCarryNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryWallAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperFrontSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperFrontSlapNoReleaseAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperLowBasketAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperLowBasketNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFullReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFullReadyToTransferNoVisionAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeToObservationZoneAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.drive.CancelDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.display.view.TeleView;
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
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.visualdisplay.AdafruitNeoPixel;
import team.techtigers.base.visualdisplay.VisualDisplaySubsystem;
import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.utils.RobotSaveState;

@SuppressWarnings("unused")
public abstract class BaseTeleOpMode extends BaseOpMode {
    private RobotState robotState;
    private IntakeSubsystem intake;
    private DropperSubsystem dropper;

    protected abstract boolean isBlue();

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        robotState = new RobotState(isBlue(), false);
        robotState.setBlockColorPreference(BlockColorPreference.ANY);
        intake = new IntakeSubsystem(hardwareMap, robotState);
        dropper = new DropperSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        AscentSubsystem ascent = new AscentSubsystem(hardwareMap, robotState);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry;

        AdafruitNeoPixel displayDriver = hardwareMap.get(AdafruitNeoPixel.class, "visual_display");
        displayDriver.initialize(224, 3);
        VisualDisplaySubsystem visualDisplaySubsystem = new VisualDisplaySubsystem(displayDriver, new TeleView(robotState));

        try {
            odometry = new GoBodometrySubsystem(hardwareMap, robotState, (Waypoint) RobotSaveState.getInstance().getState("robotCurrentPose"));
        } catch (Exception e) {
            odometry = new GoBodometrySubsystem(hardwareMap, robotState);
        }

        registerSubsystems(intake, drive, dropper, limelight,
                odometry, ascent, sensor, visualDisplaySubsystem);

        gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        // ASCENT
        Trigger startAscentTrigger =
                new Trigger(() -> gamepad1.touchpad_finger_2 || gamepad1.guide);
        Trigger isAscending = new Trigger(() -> robotState.getIsAscending());

        ManualAscentCommand manualAscentCommand = new ManualAscentCommand(robotState,
                () -> -manipulatorGamepad.getRightY(), ascent, dropper, drive);
        AscendOneLevelCommand ascendOneLevelCommand = new AscendOneLevelCommand(robotState, ascent, dropper, drive);
        StartAscentCommandGroup startAscentCommandGroup = new StartAscentCommandGroup(robotState, ascent, dropper);


        startAscentTrigger.whenActive(startAscentCommandGroup, false);

        Trigger guide = new Trigger(() -> gamepad2.guide);
        guide.whenActive(ascendOneLevelCommand);

        Trigger runningEngageAscent =
                new Trigger(() -> CommandScheduler.getInstance().isScheduled(startAscentCommandGroup));
        Trigger movingSlides = new Trigger(() -> gamepad2.right_stick_y != 0);
        isAscending.and(runningEngageAscent.negate()).and(movingSlides).whenActive(manualAscentCommand);
        Trigger driverB = driverGamepad.getGamepadButton(GamepadKeys.Button.B);
        Trigger manualDrive = new Trigger(() -> driverGamepad.getLeftX() != 0
                || driverGamepad.getLeftY() != 0
                || driverGamepad.getRightX() != 0);

        // DRIVER TODO: Split into a different method
        ManualDriveCommand manualDriveCommand = new ManualDriveCommand(drive,
                robotState, driverGamepad);
        manualDrive.whenActive(manualDriveCommand);

        DropperLowBasketAction dropperLowBasketAction = new DropperLowBasketAction(dropper, intake, robotState);
        DropperLowBasketNoTransferAction dropperLowBasketNoTransferAction = new DropperLowBasketNoTransferAction(dropper, robotState);


        CancelDriveCommand cancelDriveCommand = new CancelDriveCommand(drive);
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(cancelDriveCommand);

//        HeadingLockCommand headingLockCommand = new HeadingLockCommand(drive, robotState, driverGamepad);
        AutoSpecimenCycleAction autoSpecimenCycle = new AutoSpecimenCycleAction(drive, dropper, robotState);
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(autoSpecimenCycle);


        // MANIPULATOR

        // Intake TODO: Split into a different method

        // Commands
        IntakeToObservationZoneAction intakeToObservation =
                new IntakeToObservationZoneAction(intake, robotState);
        IntakeTuckAction tuck = new IntakeTuckAction(intake, robotState);

        ParallelCommandGroup readyToPickupManual = new ParallelCommandGroup(
                new IntakeReadyToPickupAction(intake, robotState, () -> 8),
                new DropperPreTransferAction(dropper, robotState)
        );

        IntakeFullReadyToTransferAction fullReadyToTransfer = new IntakeFullReadyToTransferAction(
                drive, intake, dropper, robotState);
        IntakeFullReadyToTransferNoVisionAction fullReadyToTransferNoVision = new IntakeFullReadyToTransferNoVisionAction(
                intake, dropper, robotState);

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
        Trigger inPrepareToPickup = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_PICKUP);
        Trigger inReadyToPickup = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_PICKUP);
        Trigger intakeInPrepareToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER);
        Trigger inReadyToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER);
//        Trigger fineBlockDetected = new Trigger(() -> robotState.getFineBlockDetectionState() == BlockDetectionState.DETECTED);
        Trigger fineBlockDetected = new Trigger(() -> robotState.isBlockDetected());
        Trigger coarseBlockDetected = new Trigger(() -> robotState.getCoarseBlockDetectionState() == BlockDetectionState.DETECTED);
        Trigger inDropperReadyToTransfer = new Trigger(() -> robotState.getDropperState() == DropperState.TRANSFER);

        // Dropper States
        DropperForwardCarryWallAction dropperForwardCarryWallAction = new DropperForwardCarryWallAction(dropper, robotState);
        DropperWallIntakeAction dropperWallIntakeAction = new DropperWallIntakeAction(dropper, intake, robotState);
        DropperWallIntakeNoTransferAction dropperWallIntakeNoTransferAction = new DropperWallIntakeNoTransferAction(dropper, robotState);
        DropperFrontSlapAction dropperFrontSlapAction = new DropperFrontSlapAction(dropper, robotState);
        DropperFrontSlapNoReleaseAction dropperFrontSlapNoReleaseAction =
                new DropperFrontSlapNoReleaseAction(dropper, robotState);
        DropperBackwardCarryNoTransferAction dropperBackwardCarryNoTransferAction = new DropperBackwardCarryNoTransferAction(dropper, robotState);
        DropperBackwardCarryAction dropperBackwardCarryAction = new DropperBackwardCarryAction(dropper, intake, robotState);
        DropperForwardCarryNoTransferAction dropperForwardCarryNoTransferAction = new DropperForwardCarryNoTransferAction(dropper, robotState);
        DropperForwardCarryAction dropperForwardCarryAction = new DropperForwardCarryAction(dropper, intake, robotState);
        DropperHighBasketNoTransferAction dropperHighBasketNoTransferAction = new DropperHighBasketNoTransferAction(dropper, robotState);
        DropperHighBasketAction dropperHighBasketAction = new DropperHighBasketAction(dropper, intake, robotState);
        DropperPreTransferAction dropperPreTransferAction = new DropperPreTransferAction(dropper, robotState);
        DropperTransferAction dropperTransferAction = new DropperTransferAction(dropper, robotState);

        // Dropper Triggers
        Trigger dpadLeft =
                manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        Trigger dpadRight =
                manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        Trigger dpadLeftAndRight =
                manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).and(manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT));
        Trigger back =
                manipulatorGamepad.getGamepadButton(GamepadKeys.Button.BACK);
        Trigger dpadUp = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        Trigger dpadDown = manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);

        Trigger forwardCarry = new Trigger(() -> robotState.getDropperState() == DropperState.FORWARD_CARRY);
        Trigger transfer =
                new Trigger(() -> robotState.getDropperState() == DropperState.PRE_TRANSFER || robotState.getDropperState() == DropperState.TRANSFER);
        Trigger wallIntake = new Trigger(() -> robotState.getDropperState() == DropperState.WALL_INTAKE);
        Trigger blockInDropper = new Trigger(() -> robotState.getBlockPosition() == RobotBlockPosition.DROPPER);
        Trigger blockInIntake = new Trigger(() -> robotState.getBlockPosition() == RobotBlockPosition.INTAKE);

        // Retract Trigger bindings
        (manualRetractTrigger.or(autoRetractTrigger)).and(inReadyToTransfer).whenActive(readyToPickupManual);
        (manualRetractTrigger.or(autoRetractTrigger)).and(inPrepareToPickup.or(inTuck).or(inReadyToPickup)).whenActive(tuck);

        // Extend Trigger Bindings
        manualExtendTrigger.and(inReadyToPickup).whenActive(fullReadyToTransferNoVision);
        manualExtendTrigger.or(autoExtendTrigger).and(inReadyToTransfer).whenActive(intakeToObservation);

        (manualExtendTrigger.or(autoExtendTrigger)).and(inTuck).and(forwardCarry).whenActive(() -> {
            readyToPickupManual.schedule();
            dropperFrontSlapAction.schedule();
        });
        (manualExtendTrigger.or(autoExtendTrigger)).and(inTuck).and(forwardCarry.negate()).whenActive(readyToPickupManual);
        (manualExtendTrigger.or(autoExtendTrigger)).and(inPrepareToPickup).whenActive(readyToPickupManual);

        Trigger velocityTrigger = new Trigger(() -> robotState.getRobotVelocity().getPoint().magnitude() < 10);
        autoExtendTrigger.and(inReadyToPickup).and(fineBlockDetected).and(velocityTrigger.negate()).whenActive(new InstantCommand(() -> {
            gamepad1.rumbleBlips(1);
            gamepad2.rumbleBlips(1);
        }));
        autoExtendTrigger.and(inReadyToPickup).and(fineBlockDetected).and(velocityTrigger).whenActive(fullReadyToTransfer);

        // Coordination between intake and dropper
        Trigger dropperSlidesLow = new Trigger(() -> dropper.getCurrentSlidePositionInches() < DropperSubsystem.SLIDES_PRE_TRANSFER_POSITION - 1);
//        dropperSlidesLow.whileActiveContinuous(() -> RobotLog.dd("tele op", "dropper slides too low"));
        manualRetractTrigger.or(autoRetractTrigger).and(inReadyToPickup.or(inPrepareToPickup)).and(dropperSlidesLow).whileActiveContinuous(dropperPreTransferAction);

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

        // Toggles Break Beam Sensor
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
                () -> {
                    robotState.setBreakBeamEnabled(false);
                    gamepad1.rumbleBlips(1);
                    gamepad2.rumbleBlips(1);
                },
                () -> {
                    robotState.setBreakBeamEnabled(true);
                    gamepad1.rumbleBlips(2);
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
                manipulatorGamepad.getLeftY() * 1.7));
        intakeSlidesTrigger.and(unsafeIntake).whileActiveOnce(unsafeIntakeSlidesCommand);

        // Manual intake rotation
        IntakeManualRotationCommand intakeManualRotationCommand =
                new IntakeManualRotationCommand(intake, manipulatorGamepad);
        Trigger intakeRotationTrigger = new Trigger(() ->
                (manipulatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 ||
                        manipulatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0)
        );
        intakeRotationTrigger.and(inReadyToPickup).whileActiveContinuous(intakeManualRotationCommand);

        // Intake claw rotation toggle to 0 or 90
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.X).and(inReadyToPickup)
                .whenActive(intake::togglePerpendicularRotation);

        // Changing Color Preference
        ChangeBlockColorPreferenceCommand changeBlockColorPreferenceCommand =
                new ChangeBlockColorPreferenceCommand(robotState, manipulatorGamepad);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                changeBlockColorPreferenceCommand);

        // Back down
        dpadDown.and(blockInIntake).whenActive(dropperTransferAction);
        dpadDown.and(blockInIntake.negate()).whenActive(dropperPreTransferAction);

        // High Basket drop
        dpadUp.and(blockInIntake).whenActive(dropperHighBasketAction);
        dpadUp.and(blockInIntake.negate()).whenActive(dropperHighBasketNoTransferAction);

        // Wall Intake
        dpadLeft.and(wallIntake.negate()).and(blockInIntake).whenActive(dropperWallIntakeAction);
        dpadLeft.and(wallIntake.negate()).and(blockInIntake.negate()).whenActive(dropperWallIntakeNoTransferAction);
        dpadRight.and(wallIntake).whenActive(dropperForwardCarryWallAction);

        // Dropper specimen movements
        back.and(forwardCarry).whenActive(dropperFrontSlapNoReleaseAction);

        dpadRight.and(forwardCarry).whenActive(dropperFrontSlapAction);
        dpadRight.and(blockInIntake).and(transfer).whenActive(dropperForwardCarryAction);
        dpadRight.and(forwardCarry.negate()).and(blockInDropper).whenActive(dropperForwardCarryNoTransferAction);

        // Low Basket drop
        driverB.and(blockInIntake).whenActive(dropperLowBasketAction);
        driverB.and(blockInIntake.negate()).whenActive(dropperLowBasketNoTransferAction);

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

        telemetry.addData("Voltage: ", robotState.getVoltage());
        telemetry.update();
        disableUpdate();
    }

    @Override
    public void justAfterStart() {
        robotState.resetTimer();
    }

    @Override
    public void update() {
//        telemetry.addData("Intake State", robotState.getIntakeState());
//        telemetry.addData("Dropper State", robotState.getDropperState());
//        telemetry.addData("Intake Slide POS",
//                intake.getCurrentSlidePositionInches());
//        telemetry.addData("Dropper Slide POS",
//                dropper.getCurrentSlidePositionInches());
//        telemetry.addData("Manual Intake?", robotState.isManualIntakeSelected());
//        telemetry.addData("Fine Block Detection State", robotState.getFineBlockDetectionState());
//        telemetry.addData("Coarse Block Detection State", robotState.getCoarseBlockDetectionState());
//        telemetry.addData("Current Block Preference", robotState.getBlockColorPreference());
////        telemetry.addData("Robot pose", robotState.getRobotCurrentPose());
////        telemetry.addData("vision intake heading", Math.toDegrees(robotState.getVisionIntakeHeading()));
////        telemetry.addLine();
//        telemetry.addData("Velocity: ", robotState.getRobotVelocity().getPoint().magnitude());
//        telemetry.addData("Heading Velocity: ", Math.toDegrees(robotState.getRobotVelocity().getHeading()));
//        telemetry.addLine();
////        telemetry.addData("Runtime: ", robotState.getRunTime());
////        telemetry.addData("Voltage: ", robotState.getVoltage());
////        telemetry.addData("Block Color: ", robotState.getIntakeBlockColor());
////        telemetry.addLine();
//        telemetry.addData("Robot X: ", robotState.getRobotCurrentPose().getX());
//        telemetry.addData("Robot Y: ", robotState.getRobotCurrentPose().getY());
//        telemetry.addData("Lateral Distance from Block", robotState.getBlockLateralFine());
//        telemetry.addData("Forward Distance from Block", robotState.getBlockForwardFine());
//        telemetry.addLine();
        telemetry.addData("Absolute Block Orientation", robotState.getAbsoluteBlockPosition().getHeading());
        telemetry.addData("Block Orientation", robotState.getBlockOrientation());
        telemetry.addData("Robot Orientation", Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));
//        telemetry.addData("Intake Claw Distance from Block", robotState.getBlockForwardCoarse());
//        telemetry.addLine();
//        telemetry.addData("Break Beam Sensor", robotState.getBlockPosition());
    }

    @Override
    public void end() {
        RobotSaveState.getInstance().setState("robotCurrentPose", robotState.getRobotCurrentPose());
    }
}
