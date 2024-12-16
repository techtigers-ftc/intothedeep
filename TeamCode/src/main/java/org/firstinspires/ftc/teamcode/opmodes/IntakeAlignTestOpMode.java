package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.CoarseAlignAction;
import org.firstinspires.ftc.teamcode.commands.actions.drive.CoarseAlignDriveAction;
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

    @Override
    public void initialize() {
        GlobalConstants.initialize(false, true);
        RobotState robotState = new RobotState();
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap, robotState);
        // TODO: input values to global constants
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState, 10.5, 3.9, 7.5,25);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        registerSubsystems(limelight, intake, drive, odometry);

        CoarseAlignAction coarseAlignAction = new CoarseAlignAction(intake, drive, robotState);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(coarseAlignAction);
    }
}
