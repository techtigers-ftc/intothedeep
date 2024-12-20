package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeCoarseAlignAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp
public class ClawCamTestOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        robotState = new RobotState();
        GamepadEx gamepad = new GamepadEx(gamepad1);

        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap, robotState);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);

        registerSubsystems(visionSubsystem, intake);

        IntakeToPickupAction intakeToPickup =
                new IntakeToPickupAction(intake, robotState);
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(intakeToPickup);

        IntakeCoarseAlignAction intakeCoarseAlignAction = new IntakeCoarseAlignAction(intake, robotState, 0.5);
        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intakeCoarseAlignAction);
    }

    @Override
    public void update() {
        telemetry.addData("Lateral Fine", robotState.getBlockLateralFine());
        telemetry.addData("Forward Fine", robotState.getBlockForwardFine());
        telemetry.addData("Orientation", robotState.getBlockOrientation());
    }
}
