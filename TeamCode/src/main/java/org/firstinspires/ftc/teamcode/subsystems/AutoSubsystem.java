package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.CloseableSubsystem;
import team.techtigers.base.statemachine.StateMachine;

/**
 * A subsystem for autonomous commands
 */
public class AutoSubsystem extends CloseableSubsystem {
    private final StateMachine<AutoState> stateMachine;
    private final RobotState robotState;

    /**
     * Constructor for the AutoSubsystem
     *
     * @param stateMachine The state machine for the autonomous command
     * @param robotState   Reference to the robot state - will be updated with the current state of
     *                     the state machine.
     */
    public AutoSubsystem(StateMachine<AutoState> stateMachine, RobotState robotState) {
        this.stateMachine = stateMachine;
        this.robotState = robotState;
    }

    @Override
    public void init() {
        stateMachine.start();
    }

    @Override
    public void periodic() {
        stateMachine.update();
        robotState.setPreviousAutoState(stateMachine.getPreviousState());
        robotState.setCurrentAutoState(stateMachine.getCurrentState());
    }
}
