package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.AutoIntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;

import team.techtigers.base.BaseOpMode;

/**
 * Test opmode for running just the VisionSubsystem
 */
@TeleOp
@SuppressWarnings("unused")
public class VisionTestOpmode extends BaseOpMode {
    RobotState robotState;
    IntakeSubsystem intake;

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        robotState = new RobotState(true, false);
        robotState.setBlockColorPreference(BlockColorPreference.ANY);
        intake = new IntakeSubsystem(hardwareMap, robotState);
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap, robotState);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);

//        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new IntakeTrackingAction(intakeSubsystem, 0.4, robotState));

        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new SequentialCommandGroup(
                new IntakePrepareToPickupAction(intake, dropper, intake::getCurrentSlidePositionInches, robotState),
                new IntakeTrackingAction(intake, 0.5, robotState),
                new WaitCommand(150),
                new AutoIntakeReadyToPickupAction(intake, dropper, robotState, null)
        ));


        registerSubsystems(vision, intake, sensor, dropper);
    }

    @Override
    public void update() {
        telemetry.addData("Forward fine", robotState.getBlockForwardFine());
        telemetry.addData("Lateral fine", robotState.getBlockLateralFine());
        telemetry.addData("Orientation", robotState.getBlockOrientation());
        telemetry.addData("Detected Fine Block Orientation", robotState.getDetectedFineBlockOrientation());
        telemetry.addData("Fine block detections state", robotState.getFineBlockDetectionState());
    }
}
