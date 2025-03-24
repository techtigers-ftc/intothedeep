package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference.ALLIANCE;
import static org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference.ANY;
import static org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference.YELLOW;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cv.AbsoluteBlockCoordinates;
import org.firstinspires.ftc.teamcode.subsystems.ControllerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.BaseOpMode;

@TeleOp(name = "Controller Subsystem Testing", group = "Tuning")
@SuppressWarnings("unused")
public class ControllerSubsystemTestOpMode extends BaseOpMode {
    RobotState robotState;
    GamepadEx driverGamepad;
    AbsoluteBlockCoordinates blockCoordinates;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);

        robotState = new RobotState(false, false);
        robotState.setBlockPosition(RobotBlockPosition.DROPPER);
        ControllerSubsystem controllerSubsystem = new ControllerSubsystem(driverGamepad, driverGamepad, robotState);
        LimelightSubsystem visionSubsystem = new LimelightSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odo = new GoBodometrySubsystem(hardwareMap, robotState);
        robotState.setCoarseCameraMode(true);
        robotState.setIntakeState(IntakeState.READY_TO_PICKUP);

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> robotState.setBlockColorPreference(ALLIANCE));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> robotState.setBlockColorPreference(ANY));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> robotState.setBlockColorPreference(YELLOW));
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> robotState.setIntakeState(IntakeState.PREPARE_TO_TRANSFER));
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> robotState.setIntakeState(IntakeState.READY_TO_PICKUP));

        registerSubsystems(controllerSubsystem, visionSubsystem, odo);
    }

    @Override
    public void update() {
        telemetry.addData("Runtime Seconds", robotState.getRunTime()/1000f);
        telemetry.addData("Color Preference", robotState.getBlockColorPreference());
        telemetry.addData("Intake State", robotState.getIntakeState());
        telemetry.addData("Block Detection State", robotState.hasBlockBeenDetected());
        telemetry.addData("Block Position", robotState.getAbsoluteBlockCoordinates().toString());
        telemetry.addData("Robot X", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Robot Y", robotState.getRobotCurrentPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(robotState.getRobotCurrentPose().getHeading()) % 360);
    }
}
