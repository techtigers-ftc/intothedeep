package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CoarseAlignDriveAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
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
        RobotState robotState = new RobotState();
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        // TODO: input actual values below
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap, robotState);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState, 10.5, 4.9, 5.5,25);
//        IntakeSubsxystem intake = new IntakeSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        registerSubsystems(limelight, drive, odometry);

        CoarseAlignDriveAction coarseAlignDriveAction = new CoarseAlignDriveAction(drive, robotState, 0.75);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(coarseAlignDriveAction);
    }
}
