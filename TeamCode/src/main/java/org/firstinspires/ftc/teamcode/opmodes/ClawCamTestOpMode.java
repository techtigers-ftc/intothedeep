package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeFineCameraPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeFineCameraAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp
public class ClawCamTestOpMode extends BaseOpMode {
    private RobotState robotState;
    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        robotState = new RobotState();
        GamepadEx gamepad = new GamepadEx(gamepad1);

        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap, robotState);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap, robotState);

        registerSubsystems(visionSubsystem, intakeSubsystem);

        IntakeFineCameraAction intakeFineCameraAction = new IntakeFineCameraAction(intakeSubsystem, robotState, 100);
        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intakeFineCameraAction);
    }

    @Override
    public void update() {
        telemetry.addData("Lateral Fine", robotState.getBlockLateralFine());
        telemetry.addData("Forward Fine", robotState.getBlockForwardFine());
        telemetry.addData("Orientation", robotState.getBlockOrientation());
    }
}
