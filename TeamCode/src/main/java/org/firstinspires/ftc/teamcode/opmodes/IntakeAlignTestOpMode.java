package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ChangeBlockColorPreferenceCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.CoarseAlignAction;
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
        GamepadEx driverGamepad = new GamepadEx(gamepad1);

        GlobalConstants.initialize(false, true);
        robotState = new RobotState();
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap, robotState);
        // TODO: input values to global constants
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState, 10.5, 4.2, 10, 25);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        registerSubsystems(limelight, intake, drive, odometry);

        CoarseAlignAction coarseAlignAction = new CoarseAlignAction(intake, drive, robotState);
        ChangeBlockColorPreferenceCommand changeBlockColorPreferenceCommand = new ChangeBlockColorPreferenceCommand(robotState);

        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(changeBlockColorPreferenceCommand);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(coarseAlignAction);
    }

    @Override
    public void update() {
        telemetry.addData("color preference", robotState.getBlockColorPreference());
    }
}
