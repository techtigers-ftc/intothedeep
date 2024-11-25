package org.firstinspires.ftc.teamcode.examples.utils;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import team.techtigers.base.BaseOpMode;
import team.techtigers.core.utils.RobotSaveState;

/**
 * Test op mode for the robot save state. This sets a value in the state
 */
@TeleOp(name = "Robot Save State SET")
public class RobotSaveStateSetOpMode extends BaseOpMode {
    private double randomId;

    @Override
    public void initialize() {
        // Generates a random number
        randomId = Math.random();

        telemetry.addData("Random ID: ", randomId);
        telemetry.update();
    }

    @Override
    protected void end() {
        // Saves the random number to the state
        RobotSaveState.getInstance().setState("randomId", randomId);
    }
}
