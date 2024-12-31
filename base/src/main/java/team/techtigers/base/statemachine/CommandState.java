package team.techtigers.base.statemachine;

import com.arcrobotics.ftclib.command.CommandBase;

/**
 * State that extends CommandBase, to be used in a state machine
 *
 * @param <T> The type of the condition, usually an enum
 */
public abstract class CommandState<T> extends CommandBase implements State<T> {
    private final String name;

    /**
     * Constructor for the CommandState
     *
     * @param name The name of the state
     */
    public CommandState(String name) {
        this.name = name;
    }

    @Override
    public String getName() {
        return name;
    }
}
