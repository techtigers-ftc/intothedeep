package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * A command that allows the driver to control the dropper slides manually
 * with no limits
 */
public class UnsafeDropperSlidesCommand extends CommandBase {
    private GamepadEx gamepad;
    private DropperSubsystem dropper;

    /**
     * Constructs a new UnsafeDropperSlidesCommand
     * @param dropper the dropper subsystem
     * @param gamepad the gamepad to control the slides
     */
    public UnsafeDropperSlidesCommand(DropperSubsystem dropper,
                                      GamepadEx gamepad) {
        this.gamepad = gamepad;
        this.dropper = dropper;
        addRequirements(dropper);
    }

    @Override
    public void execute() {
        dropper.moveSlidesRelativeUnsafe(-gamepad.getRightY()*2.5);
    }

    @Override
    public void end(boolean interrupted) {
        dropper.resetSlides();
    }
}
