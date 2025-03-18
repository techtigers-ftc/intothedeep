package org.firstinspires.ftc.teamcode.cv;

import team.techtigers.core.paths.Waypoint;

public class AbsoluteBlockCoordinates {
    private Waypoint robotPos;
    private double robotXPos;
    private double robotYPos;
    private double robotOrientation; // in radians

    private double blockLateralInches;
    private double blockForwardInches;

    public void setRobotPosition(Waypoint robotPos) {
        this.robotPos = robotPos;
        robotXPos = robotPos.getX();
        robotYPos = robotPos.getY();
        robotOrientation = robotPos.getHeading();
    }

    public void setBlockLateralInches(double blockLateralInches) {
        this.blockLateralInches = blockLateralInches;
    }

    public void setBlockForwardInches(double blockForwardInches) {
        this.blockForwardInches = blockForwardInches;
    }

    public Waypoint getBlockAbsolutePosition() {
        // Applying a rotational matrix to the block's position
        double blockX = blockLateralInches * Math.cos(robotOrientation) - blockForwardInches * Math.sin(robotOrientation);
        double blockY = blockLateralInches * Math.sin(robotOrientation) + blockForwardInches * Math.cos(robotOrientation);

        // Returning the translated coordinates + the robot's coordinates
        return new Waypoint(robotXPos + blockX, robotYPos + blockY, robotOrientation);
    }
}
