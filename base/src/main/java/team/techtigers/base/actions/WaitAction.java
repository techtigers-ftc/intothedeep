package team.techtigers.base.actions;

/**
 * Action to wait for a certain amount of time
 */
public class WaitAction implements IAction {
    private final long duration;
    private long startTime;
    private boolean isFinished;

    /**
     * Initializes a new WaitAction
     *
     * @param duration the duration to wait for, in milliseconds
     */
    public WaitAction(long duration) {
        this.duration = duration;
        isFinished = false;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void update() {
        if (System.currentTimeMillis() - startTime >= duration) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
