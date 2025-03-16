package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference.ALLIANCE;
import static org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference.ANY;
import static org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference.YELLOW;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ControllerSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

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
        ControllerSubsystem controllerSubsystem = new ControllerSubsystem(driverGamepad, manipulatorGamepad, robotState);

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> robotState.setBlockColorPreference(ALLIANCE));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> robotState.setBlockColorPreference(ANY));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> robotState.setBlockColorPreference(YELLOW));

        registerSubsystems(controllerSubsystem);
    }

    @Override
    public void update() {
        telemetry.addData("Runtime Seconds", robotState.getRunTime()/1000f);
        telemetry.addData("Color Preference", robotState.getBlockColorPreference());
    }
}
