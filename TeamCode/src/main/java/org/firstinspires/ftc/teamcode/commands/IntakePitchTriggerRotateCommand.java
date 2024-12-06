package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakePitchTriggerRotateCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final GamepadEx gamepad1;

    public IntakePitchTriggerRotateCommand(IntakeSubsystem intake, GamepadEx gamepad1) {
        this.gamepad1 = gamepad1;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setRotationRelative(gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 15);
    }
}
