package org.firstinspires.ftc.teamcode.examples.actions;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.actions.ActionRunnerCommand;

/**
 * Test op mode for the actions
 */
@TeleOp
public class ActionTestOpMode extends BaseOpMode {
    ActionRunnerCommand actionRunnerCommand;

    @Override
    public void initialize() {
        ServoSubsystem servoSubsystem = new ServoSubsystem(
                hardwareMap.servo.get("servo1"), hardwareMap.servo.get("servo2")
        );

        actionRunnerCommand =
                new ActionRunnerCommand(servoSubsystem.getServoAction(),
                        servoSubsystem);

        registerSubsystems(servoSubsystem);
    }

    @Override
    protected void justAfterStart() {
        schedule(actionRunnerCommand);
    }
}
