package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class TimeoutWaitUntilCommand extends TimeoutCommand {
    private BooleanSupplier condition;

    public TimeoutWaitUntilCommand(BooleanSupplier condition, double timeout) {
        super(timeout);
        this.condition = condition;
    }

    @Override
    public boolean isFinished() {
        return condition.getAsBoolean() || isTimeoutReached();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
