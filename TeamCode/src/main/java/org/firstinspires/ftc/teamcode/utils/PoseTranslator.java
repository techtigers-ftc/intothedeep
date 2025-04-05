package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Vector;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;

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

    /**
     * Converts a pedro pathing point to a waypoint
     *
     * @param point the pedro pathing point to convert
     * @return the waypoint
     */
    public static Waypoint pointToWaypoint(Point point) {
        return new Waypoint(point.getX(), point.getY(), 0);
    }

    /**
     * Converts a waypoint to a pedro pathing pose
     *
     * @param waypoint the waypoint to convert
     * @return the pedro pathing pose
     */
    public static Pose waypointToPedroPose(Waypoint waypoint) {
        return new Pose(waypoint.getX(), waypoint.getY(), waypoint.getHeading());
    }

    /**
     * Converts a waypoint to a pedro pathing vector
     *
     * @param waypoint the waypoint to convert
     * @return the pedro pathing vector
     */
    public static Vector waypointToPedroVector(Waypoint waypoint) {
        return new Vector(waypoint.getX(), waypoint.getY());
    }
}
