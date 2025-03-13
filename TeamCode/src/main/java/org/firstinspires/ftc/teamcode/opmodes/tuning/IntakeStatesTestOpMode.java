package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePickUpAndTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeVisionPickUpAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp
public class IntakeStatesTestOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        RobotState robotState = new RobotState(false, false);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);

        GamepadEx driverGamepad = new GamepadEx(gamepad1);

        registerSubsystems(intake, dropper, sensor, drive);

        IntakeReadyToPickupAction intakeReadyToPickupAction = new IntakeReadyToPickupAction(intake, robotState, () -> 10, () -> 0);
        IntakePrepareToTransferAction intakePrepareToTransferAction = new IntakePrepareToTransferAction(drive, intake, () -> 0, robotState);
        IntakeVisionPickUpAction intakeVisionPickUpAction1 = new IntakeVisionPickUpAction(drive, intake, dropper, robotState);
        IntakePickUpAndTransferAction intakePickUpAndTransferAction = new IntakePickUpAndTransferAction(intake, dropper, robotState);

        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intakeVisionPickUpAction1);
//        driverGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed();
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(intakeReadyToPickupAction);
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(intakePickUpAndTransferAction);
    }
}
