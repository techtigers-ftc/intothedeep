package org.firstinspires.ftc.teamcode.cv;

import com.qualcomm.robotcore.util.RobotLog;

import team.techtigers.core.paths.Waypoint;

public class AbsoluteBlockCoordinates {
    private Waypoint robotPos;
    private double robotXPos;
    private double robotYPos;
    private double robotOrientation; // in radians

    private double blockLateralInches;
    private double blockForwardInches;
    private double lockOrientation;

    public void setRobotPosition(Waypoint robotPos) {
        this.robotPos = robotPos;
        this.robotXPos = robotPos.getX();
        this.robotYPos = robotPos.getY();
        this.robotOrientation = robotPos.getHeading();
    }

    public void setBlockLateralInches(double blockLateralInches) {
        this.blockLateralInches = blockLateralInches;
    }

    public void setBlockForwardInches(double blockForwardInches) {
        this.blockForwardInches = blockForwardInches;
    }

    public void setblockOrientation(double blockOrientation) {
        this.lockOrientation = blockOrientation;
    }

    public Waypoint getBlockAbsolutePosition() {
        // Applying a rotational matrix to the block's position
        double blockX = blockForwardInches * Math.cos(robotOrientation) + blockLateralInches * Math.sin(robotOrientation);
        double blockY = blockForwardInches * Math.sin(robotOrientation) - blockLateralInches * Math.cos(robotOrientation);

        RobotLog.dd("VisionDebug", "X: %f, Y: %f, Lateral: %f, Forward: %f", blockX, blockY, blockLateralInches, blockForwardInches);


        // Returning the translated coordinates + the robot's coordinates
        return new Waypoint(robotXPos + blockX, robotYPos + blockY, lockOrientation);
    }
}
