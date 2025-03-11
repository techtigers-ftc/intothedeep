package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ChangeBlockColorPreferenceCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

/**
 * Test opmode for testing the coarse align. Has buttons to put intake in prepare to pickup, do
 * the pickup sequence, and other methods.
 */
@TeleOp
@SuppressWarnings("unused")
public class CoarseAlignTestOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        robotState = new RobotState(false, false);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap, robotState);

        robotState.setCoarseCameraMode(true);

        ChangeBlockColorPreferenceCommand changeBlockColorPreferenceCommand =
                new ChangeBlockColorPreferenceCommand(robotState, driverGamepad);
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                changeBlockColorPreferenceCommand);

        IntakePrepareToPickupAction prepareToPickupAction =
                new IntakePrepareToPickupAction(intake, dropper, robotState, 0);

        registerSubsystems(limelight, dropper, intake);
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
