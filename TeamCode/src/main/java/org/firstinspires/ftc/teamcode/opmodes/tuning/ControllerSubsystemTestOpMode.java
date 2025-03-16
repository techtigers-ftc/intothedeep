package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference.ALLIANCE;
import static org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference.ANY;
import static org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference.YELLOW;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ControllerSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.BaseOpMode;

@TeleOp
@SuppressWarnings("unused")
public class ControllerSubsystemTestOpMode extends BaseOpMode {
    RobotState robotState;

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx manipulatorGamepad = new GamepadEx(gamepad2);

        robotState = new RobotState(false, false);
        robotState.setBlockPosition(RobotBlockPosition.DROPPER);
        ControllerSubsystem controllerSubsystem = new ControllerSubsystem(driverGamepad, manipulatorGamepad, robotState);

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> robotState.setBlockColorPreference(ALLIANCE));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> robotState.setBlockColorPreference(ANY));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> robotState.setBlockColorPreference(YELLOW));
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> robotState.setIntakeState(IntakeState.PREPARE_TO_TRANSFER));
        manipulatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> robotState.setIntakeState(IntakeState.READY_TO_PICKUP));

        registerSubsystems(controllerSubsystem);
    }

    @Override
    public void update() {

        telemetry.addData("Runtime Seconds", robotState.getRunTime()/1000f);
        telemetry.addData("Color Preference", robotState.getBlockColorPreference());
        telemetry.addData("Intake State", robotState.getIntakeState());
    }
}
