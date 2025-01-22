package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ChangeBlockColorPreferenceCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeVisionPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.drive.HoldPointAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

import team.techtigers.base.BaseOpMode;

/**
 * Test opmode for running just the LimelightSubsystem
 */
@TeleOp
@Config
@SuppressWarnings("unused")
public class IntakeAlignTestOpMode extends BaseOpMode {
    public static double Y_OFFSET = 7.4;
    public static double Y_HEIGHT = 10.5;
    public static double X_OFFSET = 2.9;
    public static double DOWNWARDS_ANGLE = 25;
    private RobotState robotState;

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        robotState = new RobotState(false, false);

        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap, robotState);
        // TODO: input values to global constants
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState, Y_HEIGHT, X_OFFSET, Y_OFFSET, DOWNWARDS_ANGLE);
        // TODO: Test the following Limelight offsets: 10.5, 4.2, 10, 25
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap, robotState);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap, robotState);
        registerSubsystems(limelight, intake, dropper, drive, odometry, vision);

        // DRIVER

        ChangeBlockColorPreferenceCommand changeBlockColorPreferenceCommand =
                new ChangeBlockColorPreferenceCommand(robotState, driverGamepad);
        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(changeBlockColorPreferenceCommand);

        // MANIPULATOR
        //TODO: Model trigger behavior in normal tele opmode
        Trigger blockDetected = new Trigger(() -> robotState.getBlockDetectionState() == BlockDetectionState.DETECTED);

        IntakeVisionPickupAction intakeVisionPickupAction =
                new IntakeVisionPickupAction(intake, dropper, drive,
                        robotState, 0, null);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).and(blockDetected).whenActive(intakeVisionPickupAction);

        HoldPointAction holdPointAction = new HoldPointAction(drive, robotState,
                () -> robotState.getRobotCurrentPose().getX(),
                () -> robotState.getRobotCurrentPose().getY() - robotState.getBlockLateralCoarse(),
                () -> 0, 0.3, 2
        );
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(holdPointAction);

        // Toggles the intake claw between open and closed positions
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intake::toggleClaw);
    }

    @Override
    public void update() {
        telemetry.addData("color preference", robotState.getBlockColorPreference());
        telemetry.addData("block forward coarse", robotState.getBlockForwardCoarse());
        telemetry.addData("block forward fine", robotState.getBlockForwardFine());
        telemetry.addData("block lateral coarse", robotState.getBlockLateralCoarse());
        telemetry.addData("block lateral fine", robotState.getBlockLateralFine());
        telemetry.addData("block orientation", robotState.getBlockOrientation());
    }
}
