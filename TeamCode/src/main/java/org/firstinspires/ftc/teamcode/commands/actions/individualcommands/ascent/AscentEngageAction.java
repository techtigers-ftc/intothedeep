package org.firstinspires.ftc.teamcode.commands.actions.individualcommands.ascent;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;

public class AscentEngageAction extends CommandBase {
    private final AscentSubsystem ascent;
    private final ElapsedTime timer;
    private final double waitTime;

    public AscentEngageAction(AscentSubsystem ascent, double waitTime) {
        this.waitTime = waitTime;
        this.ascent = ascent;
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        ascent.engageAscent();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > waitTime;
    }
}
