package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ChangeBlockColorPreferenceCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.SmallCameraVisionPickup;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
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
    private RobotState robotState;

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        robotState = new RobotState(true, false);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap,
                robotState);

        driverGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            robotState.setCoarseCameraMode(!robotState.isCoarseCameraMode());
            gamepad1.rumbleBlips(1);
        });

        ChangeBlockColorPreferenceCommand changeBlockColorPreferenceCommand =
                new ChangeBlockColorPreferenceCommand(robotState, driverGamepad);
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                changeBlockColorPreferenceCommand);

        registerSubsystems(limelight);
    }

    @Override
    public void update() {
        if (robotState.isCoarseCameraMode()) {
            telemetry.addData("Forward Coarse", robotState.getBlockForwardCoarse());
            telemetry.addData("Lateral Coarse", robotState.getBlockLateralCoarse());
        } else {
            telemetry.addData("Forward Fine", robotState.getBlockForwardFine());
            telemetry.addData("Lateral Fine", robotState.getBlockLateralFine());
            telemetry.addData("Orientation", robotState.getBlockOrientation());
        }
    }
}
