package team.techtigers.base.statemachine;

/**
 * Base class for all states, which are used to run a step of the state machine
 *
 * @param <T> The type of the condition, usually an enum
 */
public interface State<T> {
    /**
     * @return The current condition of the state, usually an enum
     */
    T getCurrentCondition();

    /**
     * @return the name of the state
     */
    String getName();
}
