package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;

public class AscentCommand extends CommandBase {
    private final AscentSubsystem ascent;
    private final GamepadEx gamepad;

    public AscentCommand(AscentSubsystem ascent, GamepadEx gamepad) {
        this.ascent = ascent;
        this.gamepad = gamepad;
        addRequirements(ascent);
    }

    @Override
    public void execute() {
        double power = -gamepad.getRightY();
        ascent.powerAscent(power);
    }
}
