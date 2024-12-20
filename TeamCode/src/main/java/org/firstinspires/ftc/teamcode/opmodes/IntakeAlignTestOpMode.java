package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ChangeBlockColorPreferenceCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.CoarseAlignAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeVisionPickupAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.GlobalConstants;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

/**
 * Test opmode for running just the LimelightSubsystem
 */
@TeleOp
@SuppressWarnings("unused")
public class IntakeAlignTestOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        GlobalConstants.initialize(false, true);
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        robotState = new RobotState();

        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap, robotState);
        // TODO: input values to global constants
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState, 10.5, 3.9, 7.8, 25);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        registerSubsystems(limelight, intake, drive, odometry);

        // DRIVER

        ChangeBlockColorPreferenceCommand changeBlockColorPreferenceCommand = new ChangeBlockColorPreferenceCommand(robotState);
        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(changeBlockColorPreferenceCommand);

        // MANIPULATOR

        CoarseAlignAction coarseAlignAction = new CoarseAlignAction(intake, drive, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(coarseAlignAction);

        IntakeVisionPickupAction intakeVisionPickupAction = new IntakeVisionPickupAction(intake, drive, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(intakeVisionPickupAction);
    }

    @Override
    public void update() {
        telemetry.addData("color preference", robotState.getBlockColorPreference());
        telemetry.addData("block forward coarse", robotState.getBlockForwardCoarse());
        telemetry.addData("block lateral coarse", robotState.getBlockLateralCoarse());
        telemetry.addData("block orientation", robotState.getBlockOrientation());
    }
}
