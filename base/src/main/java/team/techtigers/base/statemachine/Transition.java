package team.techtigers.base.statemachine;

/**
 * Represents a condition that must be met for a state to transition to the next state
 *
 * @param <T> The type of the condition, usually an enum
 */
public class Transition<T> {
    private final T endCondition;
    private final String nextState;

    /**
     * Initializes a new Transition
     *
     * @param endCondition the condition where a state should transition
     * @param nextState    the name of the next state
     */
    public Transition(T endCondition, String nextState) {
        this.endCondition = endCondition;
        this.nextState = nextState;
    }

    /**
     * @param currentCondition the current condition of the state
     * @return if the condition is met
     */
    public boolean isFinished(T currentCondition) {
        return endCondition == currentCondition;
    }

    /**
     * @return the name of the next state
     */
    public String getNextState() {
        return nextState;
    }
}
