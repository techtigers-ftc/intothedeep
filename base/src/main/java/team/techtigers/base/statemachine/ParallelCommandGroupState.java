package team.techtigers.base.statemachine;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

/**
 * State that extends ParallelCommandGroup, to be used in a state machine
 *
 * @param <T> The type of the condition, usually an enum
 */
public abstract class ParallelCommandGroupState<T> extends ParallelCommandGroup implements State<T> {
    private final String name;

    /**
     * Constructor for the ParallelCommandGroupState
     *
     * @param name The name of the state
     */
    public ParallelCommandGroupState(String name) {
        this.name = name;
    }

    @Override
    public String getName() {
        return name;
    }
}
