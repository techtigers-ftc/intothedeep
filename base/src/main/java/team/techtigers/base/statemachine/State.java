package team.techtigers.base.statemachine;

import com.arcrobotics.ftclib.command.CommandBase;

/**
 * Base class for all states, which are used to run a step of the state machine
 *
 * @param <T> The type of the condition, usually an enum
 */
public abstract class State<T> extends CommandBase {
    private final String name;

    /**
     * Initializes a new state
     * @param name the name of the state
     */
    public State(String name) {
        super();
        this.name = name;
    }

    /**
     * @return The current condition of the state, usually an enum
     */
    public abstract T getCurrentCondition();

    /**
     * @return the name of the state
     */
    public String getName() {
        return name;
    }
}
