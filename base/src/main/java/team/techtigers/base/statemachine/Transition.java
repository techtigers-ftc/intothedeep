package team.techtigers.base.statemachine;

/**
 * Represents a condition that must be met for a state to transition to the next state
 *
 * @param <T> The type of the condition, usually an enum
 */
public class Transition<T> {
    private final T endCondition;
    private final State<T> nextState;

    /**
     * Initializes a new Transition
     *
     * @param endCondition the condition where a state should transition
     * @param nextState    the next state to transition to
     */
    public Transition(T endCondition, State<T> nextState) {
        this.endCondition = endCondition;
        this.nextState = nextState;
    }

    /**
     * @param currentCondition the current condition of the state
     * @return if the condition is met
     */
    public boolean meetsCondition(T currentCondition) {
        return endCondition == currentCondition;
    }

    /**
     * @return the next state to transition to
     */
    public State<T> getNextState() {
        return nextState;
    }
}
