package org.firstinspires.ftc.teamcode.cv;

import team.techtigers.core.paths.Waypoint;

public class AbsoluteBlockCoordinates {
    private Waypoint robotPos;
    private double blockLateralInches;
    private double blockForwardInches;

    public AbsoluteBlockCoordinates() {
    }

    public void setRobotPosition(Waypoint robotPos) {
        this.robotPos = robotPos;
    }

    public void setBlockLateralInches(double blockLateralInches) {
        this.blockLateralInches = blockLateralInches;
    }

    public void setBlockForwardInches(double blockForwardInches) {
        this.blockForwardInches = blockForwardInches;
    }

//    public Waypoint getBlockAbsolutePosition() {
//
//    }
}
