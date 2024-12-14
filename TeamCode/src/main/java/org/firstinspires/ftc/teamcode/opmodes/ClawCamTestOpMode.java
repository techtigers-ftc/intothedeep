package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        RobotState robotState = new RobotState();
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);

        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap, robotState);
//        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
//        registerSubsystems(visionSubsystem, intake);
        registerSubsystems(visionSubsystem);

//        IntakeFineCameraAction intakeFineCamera = new IntakeFineCameraAction(intake, robotState, 50);
//        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intakeFineCamera);

//        IntakeFineCameraPickupAction intakePickupFineCamera = new IntakeFineCameraPickupAction(intake, robotState);
//        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(intakePickupFineCamera);
    }
}
