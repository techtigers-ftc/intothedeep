package org.firstinspires.ftc.teamcode.examples.utils;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import team.techtigers.base.BaseOpMode;
import team.techtigers.core.utils.RobotSaveState;

/**
 * Test op mode for the robot save state. This reads a value in the state
 */
@TeleOp(name = "Robot Save State READ")
public class RobotSaveStateReadOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        // This opmode only works after running the RobotSaveStateSetOpMode
        try {
            telemetry.addData("ID: ",
                    RobotSaveState.getInstance().getState("randomId"));
        } catch (Exception e) {
            telemetry.addData("ID: ", "No ID found");
        }
        telemetry.update();
    }

    @Override
    protected void end() {
        // This opmode resets at the end
        RobotSaveState.reset();
    }
}
