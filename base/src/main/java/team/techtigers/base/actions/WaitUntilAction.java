package team.techtigers.base.actions;

import java.util.function.BooleanSupplier;

/**
 * Action that waits until a condition is met
 */
public class WaitUntilAction implements IAction {
    private final BooleanSupplier condition;

    /**
     * Initializes a new WaitUntilAction
     *
     * @param condition The condition to wait for. When it becomes true, the action will finish
     */
    public WaitUntilAction(BooleanSupplier condition) {
        if (condition == null) {
            throw new IllegalArgumentException("Condition cannot be null");
        }
        this.condition = condition;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return condition.getAsBoolean();
    }
}
