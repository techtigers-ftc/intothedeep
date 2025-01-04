package team.techtigers.base.statemachine;

/**
 * Builder class for creating transitions in a state machine
 *
 * @param <T> The type of the condition, usually an enum
 **/
public class TransitionBuilder<T> {
    private final StateMachine<T> stateMachine;
    private final State<T> currentState;
    private State<T> nextState;

    /**
     * Initializes a new TransitionBuilder
     *
     * @param stateMachine the state machine to add the transition to
     * @param currentState the current state
     */
    public TransitionBuilder(StateMachine<T> stateMachine,
                             State<T> currentState) {
        this.stateMachine = stateMachine;
        this.currentState = currentState;
    }

    /**
     * Adds a condition to the transition
     *
     * @param condition the condition that must be met for the state to transition
     * @return the state machine to allow for method chaining
     */
    public StateMachine<T> when(T condition) {
        if (nextState == null) {
            throw new IllegalStateException("Condition must be set (use to)");
        }

        stateMachine.addCondition(currentState, new Transition<>(condition,
                nextState));

        return stateMachine;
    }

    /**
     * Adds the next state to the transition
     *
     * @param nextState the next state to transition to
     * @return the transition builder to allow for method chaining
     */
    public TransitionBuilder<T> to(State<T> nextState) {
        this.nextState = nextState;

        return this;
    }
}
