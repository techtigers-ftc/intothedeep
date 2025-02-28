package team.techtigers.base.statemachine;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.robot.RobotState;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Objects;

/**
 * State machine that runs a series of states and transitions between them based on conditions
 *
 * @param <T> the type of the conditions used by the machine to transition
 *            between states
 */
public class StateMachine<T> {
    private final ArrayList<State<T>> stateList;
    private final HashMap<String, ArrayList<Transition<T>>> transitionMap;
    private State<T> currentState;
    private State<T> previousState;
    private ArrayList<Transition<T>> currentTransitions;
    private RobotState robotState;

    /**
     * Initializes a new StateMachine
     */
    public StateMachine() {
        stateList = new ArrayList<>();
        transitionMap = new HashMap<>();
        currentState = null;
        previousState = null;
        currentTransitions = null;
    }

    /**
     * Adds a state to the state machine
     *
     * @param state the state for the state
     * @return the state machine to allow for method chaining
     */
    public StateMachine<T> addState(State<T> state) {
        if (transitionMap.containsKey(state.getName())) {
            throw new IllegalArgumentException("State: " + state.getName() + " already exists");
        }
        stateList.add(state);
        transitionMap.put(state.getName(), new ArrayList<>());

        return this;
    }

    /**
     * Adds a transition to the state machine
     *
     * @param currentState the current state
     * @return the transition builder to allow for method chaining
     */
    public TransitionBuilder<T> from(State<T> currentState) {
        return new TransitionBuilder<>(this, currentState);
    }

    /**
     * Adds a condition to the state machine. This should only be
     * used in the TransitionBuilder, other users should use the from method.
     *
     * @param currentState the current state
     * @param transition   the transition to add
     */
    void addCondition(State<T> currentState, Transition<T> transition) {
        if (!stateList.contains(currentState)) {
            throw new IllegalArgumentException("State: " + currentState.getName() + " does not exist");
        }
        if (!stateList.contains(transition.getNextState())) {
            throw new IllegalArgumentException("State: " + transition.getNextState().getName() + " does not exist");
        }

        Objects.requireNonNull(transitionMap.get(currentState.getName())).add(transition);
    }

    /**
     * Single line utility method to add a transition to the state machine.
     *
     * @param fromState the state to transition from
     * @param toState   the state to transition to
     * @param condition the condition that must be met for the transition
     * @return the state machine to allow for method chaining
     */
    public StateMachine<T> addTransition(State<T> fromState, State<T> toState,
                                         T condition) {
        addCondition(fromState, new Transition<>(condition, toState));
        return this;
    }

    /**
     * Sets the current state of the state machine that will be run.
     * Note: In the opmode, this should be the last call in the chain
     *
     * @param state the state to be set as the running state
     */
    public void setCurrentState(State<T> state) {
        if (!stateList.contains(state)) {
            throw new IllegalArgumentException("State: " + state + " does not exist");
        }

        previousState = currentState;
        currentState = state;

        currentTransitions = transitionMap.get(currentState.getName());
    }

    /**
     * Starts the state machine
     */
    public void start() {
        if (currentState == null) {
            throw new IllegalStateException("No first state set");
        }

        CommandScheduler.getInstance().schedule(currentState);
    }

    /**
     * Updates the state machine
     */
    public void update() {
        if (currentState == null) {
            throw new IllegalStateException("No current state set");
        }
        if (currentTransitions == null) {
            throw new IllegalStateException("No transitions set for state: " + this.currentState);
        }
        T currentCondition = currentState.getCurrentCondition();

        for (Transition<T> transition : currentTransitions) {
            if (transition.meetsCondition(currentCondition)) {
                CommandScheduler.getInstance().cancel(currentState);
                setCurrentState(transition.getNextState());
                CommandScheduler.getInstance().schedule(currentState);
                break;
            }
        }
    }

    /**
     * @return the name of the current state running in the state machine
     */
    public String getCurrentState() {
        if(currentState != null ) {
            return currentState.getName();
        }
        return "";
    }

    /**
     * @return the name of the previous state of the state machine
     */
    public String getPreviousState() {
        if(previousState != null) {
            return previousState.getName();
        }
        return "";
    }
}
