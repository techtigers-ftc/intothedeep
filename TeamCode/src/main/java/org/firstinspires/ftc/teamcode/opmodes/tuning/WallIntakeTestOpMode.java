package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperFrontSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryWallAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

import team.techtigers.base.BaseOpMode;

/**
 * An opmode to test the capabilities of the dropper subsystem, including the slides, arm, and claw
 */
@TeleOp(name = "Not 3Dropper Tuning OpMode", group = "Tuning")
public class WallIntakeTestOpMode extends BaseOpMode {
    private DropperSubsystem dropperSubsystem;
    private RobotState robotState;
    private IntakeSubsystem intakeSubsystem;
    private DriveSubsystem drive;

    @Override
    public void initialize() {
        robotState = new RobotState(true, false);
        dropperSubsystem = new DropperSubsystem(hardwareMap, robotState);
        drive = new DriveSubsystem(hardwareMap, robotState);
        registerSubsystems(dropperSubsystem, drive);

        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Claw
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.closeClaw();
        }));
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> {
            dropperSubsystem.openClaw();
        }));

        //Wall Intake
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new DropperWallIntakeAction(dropperSubsystem, robotState, drive));
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new DropperForwardCarryWallAction(dropperSubsystem, robotState, drive));
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new DropperPreTransferAction(dropperSubsystem, robotState));
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new DropperFrontSlapAction(dropperSubsystem, robotState));

        ManualDriveCommand driveCommand = new ManualDriveCommand(drive, driverGamepad);
        drive.setDefaultCommand(driveCommand);

        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                () -> robotState.setCurrentGear(DriveGears.ENGAGED));
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                () -> robotState.setCurrentGear(DriveGears.NOT_ENGAGED));

    }

    @Override
    public void update() {
        double currentPos = dropperSubsystem.getCurrentSlidePositionInches();
        double expectedPos = dropperSubsystem.getTargetPositionInches();
        double error = expectedPos - currentPos;

        telemetry.addData("Current slide position (inches)", currentPos);
        telemetry.addData("Expected slide position (inches)", expectedPos);
        telemetry.addData("Slide position error (inches)", error);
        telemetry.addLine();
        telemetry.addData("Claw rotation angle", dropperSubsystem.getRotation());
        telemetry.addData("Claw diff pitch", dropperSubsystem.getPitch());
        telemetry.addLine();
        telemetry.addData("Left slide current draw:", dropperSubsystem.getSlideCurrentLeft());
        telemetry.addData("Right slide current draw:", dropperSubsystem.getSlideCurrentRight());
        telemetry.addData("Invalid dropper state error:", robotState.hasError(RobotError.INVALID_DROPPER_POSITION));
    }
}
