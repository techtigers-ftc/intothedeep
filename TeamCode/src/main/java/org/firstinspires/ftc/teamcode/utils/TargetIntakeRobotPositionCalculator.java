package org.firstinspires.ftc.teamcode.utils;

import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.paths.geometry.Point;

/**
 * Given a robot position and a block position, the methods in this class will return the correct
 * robot position and slide distance that the robot and slides should go to in order to pick up
 * the block.
 */
public class TargetIntakeRobotPositionCalculator {
    public static double[] getTargetIntakePositionFine(Waypoint robotPose, Waypoint blockPose) {
        Point robotVector = robotPose.getPoint();
        Point blockVector = blockPose.getPoint();
        Point robotToBlockVector = blockVector.minus(robotVector);

        double distance = robotVector.dist(blockVector);
        double angleBetween = robotPose.getHeading() - Math.atan2(robotToBlockVector.getY(), robotToBlockVector.getX());

        double forwardDistance = distance * Math.cos(angleBetween);
        double lateralDistance = distance * Math.sin(angleBetween);

        Waypoint robotTarget = new Waypoint(
                robotPose.getX() + Math.sin(robotPose.getHeading()) * lateralDistance,
                robotPose.getY() - Math.cos(robotPose.getHeading()) * lateralDistance,
                robotPose.getHeading()
        );

        return new double[]{robotTarget.getX(), robotTarget.getY(), robotTarget.getHeading(), forwardDistance};
    }
}
