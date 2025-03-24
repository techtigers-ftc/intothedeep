package org.firstinspires.ftc.teamcode.utils;

import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.paths.geometry.Point;

/**
 * Given a robot position and a block position, the methods in this class will return the correct
 * robot position and slide distance that the robot and slides should go to in order to pick up
 * the block.
 */
public class TargetRobotPoseCalculator {
    /**
     * Using the absolute positions of a block and the robot, calculates the target position the
     * robot needs to go to in order to pick up the block.
     *
     * @param robotPose the robot's absolute position
     * @param blockPose the block's absolute position
     * @return the target positions and distances the robot should go to in order to pick up the block
     */
    public static double[] getTargetIntakePosition(Waypoint robotPose, Waypoint blockPose) {
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

        double clawOrientation = (-blockPose.getHeading() + Math.toDegrees(robotPose.getHeading()) + 90);
        clawOrientation = ((clawOrientation % 180) + 180) % 180;

        return new double[]{robotTarget.getX(), robotTarget.getY(), robotTarget.getHeading(), forwardDistance, clawOrientation};
    }
}
