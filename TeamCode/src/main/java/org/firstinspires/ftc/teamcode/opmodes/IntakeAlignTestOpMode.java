package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CoarseAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
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
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState, 10.5, 4.9, 5.5,25);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        registerSubsystems(limelight, intake, drive);

        CoarseAlignCommand coarseAlignCommand = new CoarseAlignCommand(drive, intake, robotState, 0.5);
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(coarseAlignCommand);
    }
}
