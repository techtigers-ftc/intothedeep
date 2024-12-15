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
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);

        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap, robotState);
        registerSubsystems(visionSubsystem);
    }

    @Override
    public void update() {
        telemetry.addData("Sample X", robotState.getBlockLateralFine());
        telemetry.addData("Sample Y", robotState.getBlockForwardFine());
        telemetry.addData("Orientation", robotState.getBlockOrientation());
    }
}
