package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.IntakeTransferCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

public class ScrimmageTeleOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        RobotState robotState = new RobotState();
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        registerSubsystems(intake);

//        gamepadEx.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(COMMAD);
//        gamepadEx.getGamepadButton(GamepadKeys.Button.X).whenInactive(OTHER COMMAND)

        Trigger slidesTrigger = new Trigger(() ->
                gamepadEx.getLeftY() != 0
        );

        slidesTrigger.whileActiveContinuous(() -> intake.moveSlidesRelative(gamepadEx.getLeftY() * 2));

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new IntakeTransferCommandGroup(intake));
    }
}
