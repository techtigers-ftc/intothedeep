package org.firstinspires.ftc.teamcode.examples.utils;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import team.techtigers.base.BaseOpMode;

/**
 * Test op mode for global state
 */
@TeleOp
public class GlobalStateTestOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        robotState = new RobotState();

        // Should be 0
        telemetry.addData("Intake Height: ", robotState.getIntakeHeight());
        // Should be false
        telemetry.addData("Is Running: ", robotState.isRunning());
        telemetry.update();
    }

    @Override
    protected void justAfterStart() {
        robotState.setIntakeHeight(10); // Sets to 10
        robotState.setIsRunning(true); // Sets to true
        robotState.resetTimer();
    }

    @Override
    protected void update() {
        // Should be 10
        telemetry.addData("Intake Height: ", robotState.getIntakeHeight());
        // Should be true
        telemetry.addData("Is Running: ", robotState.isRunning());
        // Constantly increasing
        telemetry.addData("Time: ", robotState.getRunTime());

        // NOTE: BaseOpMode gives a telemetry.update() automatically, so there
        // is no need for one here
    }
}
