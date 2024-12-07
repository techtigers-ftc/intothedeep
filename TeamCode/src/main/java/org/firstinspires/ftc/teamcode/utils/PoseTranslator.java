package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedropathing.util.Pose;

import team.techtigers.core.paths.Waypoint;

/**
 * Class for translating between different forms of poses
 */
public class PoseTranslator {
    /**
     * Converts a waypoint to a pose
     *
     * @param waypoint the waypoint to convert
     * @return the pedro pathing pose
     */
    public static Pose waypointToPose(Waypoint waypoint) {
        return new Pose(waypoint.getX(), waypoint.getY(), waypoint.getHeading());
    }

    /**
     * Converts a pose to a waypoint
     *
     * @param pose the pedro pathing pose to convert
     * @return the waypoint
     */
    public static Waypoint poseToWaypoint(Pose pose) {
        return new Waypoint(pose.getX(), pose.getY(), pose.getHeading());
    }

    /**
     * Converts a waypoint to a vector
     *
     * @param waypoint the waypoint to convert
     * @return the pedro pathing vector
     */
    public static Vector waypointToVector(Waypoint waypoint) {
        return new Vector(waypoint.getX(), waypoint.getY());
    }
}
