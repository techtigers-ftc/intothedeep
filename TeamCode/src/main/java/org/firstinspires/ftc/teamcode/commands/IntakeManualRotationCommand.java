package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * A command that allows manual control of the intake rotation
 */
public class IntakeManualRotationCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final GamepadEx gamepad1;

    /**
     * Initializes a new IntakeManualRotationCommand
     *
     * @param intake   the intake subsystem
     * @param gamepad1 the gamepad to use to control the rotation of the intake
     */
    public IntakeManualRotationCommand(IntakeSubsystem intake, GamepadEx gamepad1) {
        this.gamepad1 = gamepad1;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setClawRotationRelative(-(gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) * 2.5);
    }
}
