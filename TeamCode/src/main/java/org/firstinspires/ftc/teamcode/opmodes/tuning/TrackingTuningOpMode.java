package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.UnsafeIntakeSlidesCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFullReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.display.view.TeleView;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.visualdisplay.AdafruitNeoPixel;
import team.techtigers.base.visualdisplay.VisualDisplaySubsystem;

@TeleOp
public class TrackingTuningOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        GamepadEx gamepad = new GamepadEx(gamepad1);

        RobotState robotState = new RobotState(true, false);
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, robotState);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap, robotState);
        DropperSubsystem dropperSubsystem = new DropperSubsystem(hardwareMap, robotState);
        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap, robotState);
        SensorSubsystem sensorSubsystem = new SensorSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometrySubsystem =
                new GoBodometrySubsystem(hardwareMap, robotState);

        AdafruitNeoPixel displayDriver = hardwareMap.get(AdafruitNeoPixel.class, "visual_display");
        displayDriver.initialize(224, 3);
        VisualDisplaySubsystem visualDisplaySubsystem = new VisualDisplaySubsystem(displayDriver, new TeleView(robotState));

        registerSubsystems(driveSubsystem, intakeSubsystem, dropperSubsystem,
                limelightSubsystem, visualDisplaySubsystem, sensorSubsystem, odometrySubsystem);

        IntakeTrackingAction trackingAction = new IntakeTrackingAction(
                intakeSubsystem,
                robotState
        );

        IntakeFullReadyToTransferAction fullReadyToTransferAction = new IntakeFullReadyToTransferAction(
                driveSubsystem,
                intakeSubsystem,
                dropperSubsystem,
                robotState
        );

        IntakeTuckAction tuckAction = new IntakeTuckAction(
                intakeSubsystem,
                robotState
        );

        IntakeReadyToPickupAction readyToPickupAction = new IntakeReadyToPickupAction(
                intakeSubsystem,
                robotState,
                () -> 0
        );

        Trigger rightBumper = gamepad.getGamepadButton(
                GamepadKeys.Button.RIGHT_BUMPER
        );

        Trigger leftBumper = gamepad.getGamepadButton(
                GamepadKeys.Button.LEFT_BUMPER
        );

        Trigger a = gamepad.getGamepadButton(
                GamepadKeys.Button.A
        );

        Trigger inTuck = new Trigger(() -> robotState.getIntakeState() == IntakeState.TUCK);
        Trigger inPrepareToPickup = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_PICKUP);
        Trigger inReadyToPickup = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_PICKUP);
        Trigger intakeInPrepareToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER);
        Trigger inReadyToTransfer = new Trigger(() -> robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER);
//        Trigger fineBlockDetected = new Trigger(() -> robotState.getFineBlockDetectionState() == BlockDetectionState.DETECTED);
        Trigger fineBlockDetected = new Trigger(robotState::isBlockDetected);
        Trigger coarseBlockDetected = new Trigger(() -> robotState.getCoarseBlockDetectionState() == BlockDetectionState.DETECTED);

        rightBumper.and(inTuck.or(inReadyToTransfer)).whenActive(readyToPickupAction);
        rightBumper.and(inReadyToPickup).and(fineBlockDetected).whenActive(fullReadyToTransferAction);

        leftBumper.and(inTuck).and(inReadyToPickup).whenActive(tuckAction);

        a.and(inReadyToPickup).whenActive(trackingAction);

        // Controls intake slides
        UnsafeIntakeSlidesCommand unsafeIntakeSlidesCommand =
                new UnsafeIntakeSlidesCommand(intakeSubsystem, gamepad);

        Trigger unsafeIntake = gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
        Trigger intakeSlidesTrigger = new Trigger(() ->
                gamepad.getLeftY() != 0
        );

        intakeSlidesTrigger.and(unsafeIntake.negate()).whileActiveContinuous(() -> intakeSubsystem.moveSlidesRelative(
                gamepad.getLeftY() * 1.7));
        intakeSlidesTrigger.and(unsafeIntake).whileActiveOnce(unsafeIntakeSlidesCommand);

    }
}
