package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.AutoSpecimenCycleAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.drive.CancelDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;
import team.techtigers.core.paths.Waypoint;

@TeleOp
public class AutoSpecimenCycleTestOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        robotState = new RobotState(false, false);
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap, robotState);
//        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap, robotState, new Waypoint(113,13, Math.toRadians(90)));
//        registerSubsystems(drive, dropper, intake, sensor, odometry);
        registerSubsystems(drive, dropper, sensor, odometry);

        AutoSpecimenCycleAction autoSpecimenCycle = new AutoSpecimenCycleAction(drive, dropper, odometry, robotState);
        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(autoSpecimenCycle);

        DropperWallIntakeNoTransferAction dropperWallIntakeAction = new DropperWallIntakeNoTransferAction(dropper, robotState);
        driverGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(dropperWallIntakeAction);

        DropperOpenAction openAction = new DropperOpenAction(dropper, 100);
        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(openAction);

        CancelDriveCommand cancelDriveCommand = new CancelDriveCommand(
                drive
        );
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(cancelDriveCommand);
    }

    @Override
    public void update(){
        telemetry.addData("X: ", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Y: ", robotState.getRobotCurrentPose().getY());
        telemetry.addData("Heading (deg): ", robotState.getRobotCurrentPose().getHeading());
    }
}
