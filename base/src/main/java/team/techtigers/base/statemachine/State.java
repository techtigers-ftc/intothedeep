package team.techtigers.base.statemachine;

/**
 * Interface for all states, which are used to run a step of the state machine
 *
 * @param <T> The type of the condition, usually an enum
 */
public interface State<T> {
    /**
     * Returns the current condition of the state every update cycle
     */
    T getCurrentCondition();

    /**
     * Returns the name of the state
     */
    String getName();
}
