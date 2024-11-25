package team.techtigers.base.statemachine;

import com.arcrobotics.ftclib.command.CommandScheduler;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * State machine that runs a series of states and transitions between them based on conditions
 */
public class StateMachine {
    private final HashMap<String, State> states;
    private final HashMap<String, ArrayList<Transition>> conditions;
    private String currentState;

    /**
     * Initializes a new StateMachine
     */
    public StateMachine() {
        states = new HashMap<>();
        conditions = new HashMap<>();
        currentState = null;
    }

    /**
     * Adds a state to the state machine
     *
     * @param state the state for the state
     * @return the state machine to allow for method chaining
     */
    public StateMachine addState(State state) {
        states.put(state.getName(), state);

        return this;
    }

    /**
     * Adds a transition to the state machine
     *
     * @param currentState the name of the current state
     * @return the transition builder to allow for method chaining
     */
    public TransitionBuilder from(String currentState) {
        return new TransitionBuilder(this, currentState);
    }

    /**
     * Adds a condition to the state machine. This should only be
     * used in the TransitionBuilder, other users should use the from method.
     *
     * @param currentState the name of the condition
     * @return the state machine to allow for method chaining
     */
    void addCondition(String currentState, Transition transition) {
        if (!conditions.containsKey(currentState)) {
            conditions.put(currentState, new ArrayList<>());
        }
        conditions.get(currentState).add(transition);
    }

    /**
     * Sets the first state of the state machine that will be run
     *
     * @param stateName the name of the first state
     * @return the state machine to allow for method chaining
     */
    public StateMachine setFirstState(String stateName) {
        currentState = stateName;
        if (!states.containsKey(stateName)) {
            throw new IllegalArgumentException("State: " + stateName + " does not exist");
        }

        return this;
    }

    /**
     * Starts the state machine
     */
    public void start() {
        if (currentState == null) {
            throw new IllegalStateException("No first state set");
        }

        CommandScheduler.getInstance().schedule(states.get(currentState));
    }

    /**
     * Updates the state machine
     */
    public void update() {
        if (conditions.get(currentState) != null) {
            State stateExecutor = states.get(currentState);
            for (Transition transition : conditions.get(currentState)) {
                if (transition.isFinished(stateExecutor.getCurrentCondition())) {
                    CommandScheduler.getInstance().cancel(states.get(currentState));

                    currentState = transition.getNextState();
                    if (!states.containsKey(currentState)) {
                        throw new IllegalArgumentException("State: " + currentState + " does not exist");
                    }

                    CommandScheduler.getInstance().schedule(states.get(currentState));
                    break;
                }
            }
        }
    }
}
