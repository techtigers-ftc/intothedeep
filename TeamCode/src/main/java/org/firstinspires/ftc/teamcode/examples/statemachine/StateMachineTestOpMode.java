package org.firstinspires.ftc.teamcode.examples.statemachine;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.statemachine.StateMachine;

/**
 * Test op mode for the state machine
 */
@TeleOp
public class StateMachineTestOpMode extends BaseOpMode {
    StateMachine stateMachine;

    @Override
    public void initialize() {
        stateMachine = new StateMachine();

        StateOne stateOne = new StateOne("stateOne");
        StateTwo stateTwo = new StateTwo("stateTwo");

        stateMachine
                .addState(stateOne)
                .addState(stateTwo)
                .from("stateOne")
                .to("stateTwo")
                .when(Condition.ENDED)
                .setFirstState("stateOne");

    }

    @Override
    protected void justAfterStart() {
        stateMachine.start();
    }

    @Override
    protected void update() {
        stateMachine.update();
    }
}
