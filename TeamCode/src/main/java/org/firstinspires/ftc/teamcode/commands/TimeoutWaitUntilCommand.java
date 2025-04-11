package org.firstinspires.ftc.teamcode.commands;

import java.util.function.BooleanSupplier;

/**
 * A command that waits until a condition is met or a timeout is reached.
 */
public class TimeoutWaitUntilCommand extends TimeoutCommand {
    private BooleanSupplier condition;

    /**
     * Constructor for the TimeoutWaitUntilCommand
     *
     * @param condition The condition to wait for
     * @param timeout   The timeout in seconds
     */
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
