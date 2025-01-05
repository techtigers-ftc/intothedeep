package org.firstinspires.ftc.teamcode.subsystems;

import team.techtigers.base.CloseableSubsystem;
import team.techtigers.base.statemachine.StateMachine;

/**
 * A subsystem for autonomous commands
 */
public class AutoSubsystem extends CloseableSubsystem {
    private StateMachine stateMachine;

    /**
     * Constructor for the AutoSubsystem
     *
     * @param stateMachine The state machine for the autonomous command
     */
    public AutoSubsystem(StateMachine stateMachine) {
        this.stateMachine = stateMachine;
    }

    @Override
    public void init() {
        stateMachine.start();
    }

    @Override
    public void periodic() {
        stateMachine.update();
    }
}
