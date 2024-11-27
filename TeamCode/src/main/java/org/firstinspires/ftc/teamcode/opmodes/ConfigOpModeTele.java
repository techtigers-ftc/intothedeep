package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

/**
 * Base class for all Tele opmodes
 */
public abstract class ConfigOpModeTele extends BaseOpMode {
    protected RobotState robotState;

    @Override
    public void initialize() {
        robotState = new RobotState();
    }
}
