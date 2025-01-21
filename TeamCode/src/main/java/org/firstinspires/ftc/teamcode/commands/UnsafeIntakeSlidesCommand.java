package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * A command that allows the driver to control the intake slides manually
 * with no limits
 */
public class UnsafeIntakeSlidesCommand extends CommandBase {
    private GamepadEx gamepad;
    private IntakeSubsystem intake;

    /**
     * Constructs a new UnsafeIntakeSlidesCommand
     * @param intake the intake subsystem
     * @param gamepad the gamepad to control the slides
     */
    public UnsafeIntakeSlidesCommand(IntakeSubsystem intake,
                                     GamepadEx gamepad) {
        this.gamepad = gamepad;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.moveSlidesRelativeUnsafe(gamepad.getLeftY()*4);
    }

    @Override
    public void end(boolean interrupted) {
        intake.resetSlides();
    }
}
