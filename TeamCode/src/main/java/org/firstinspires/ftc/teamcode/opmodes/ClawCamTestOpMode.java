package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp
public class ClawCamTestOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        robotState = new RobotState(false, false);
        GamepadEx gamepad = new GamepadEx(gamepad1);

        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap, robotState);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);

        registerSubsystems(visionSubsystem, intake);
    }

    @Override
    public void update() {
        telemetry.addData("Lateral Fine", robotState.getBlockLateralFine());
        telemetry.addData("Forward Fine", robotState.getBlockForwardFine());
        telemetry.addData("Orientation", robotState.getBlockOrientation());
    }
}
