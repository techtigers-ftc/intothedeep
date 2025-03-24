package team.techtigers.core.paths;

import team.techtigers.core.paths.geometry.Point;


/**
 * Represents a waypoint in a trajectory. A waypoint is a point in a trajectory that the robot must
 * pass through.
 */
public class Waypoint {
    private final Point point;
    private final double heading;

    /**
     * Constructs a new Waypoint using a point input
     *
     * @param point   The pose to be saved inside the class
     * @param heading The heading
     */
    public Waypoint(Point point, double heading) {
        this.point = point;
        this.heading = heading;
    }

    /**
     * Constructs a new Waypoint using x, y, and heading inputs
     *
     * @param x       The x coordinate of the waypoint
     * @param y       The y coordinate of the waypoint
     * @param heading The heading of the waypoint
     */
    public Waypoint(double x, double y, double heading) {
        this(new Point(x, y), heading);
    }

    /**
     * Constructs a new Waypoint using x and y inputs, assuming heading is 0
     *
     * @param x The x coordinate of the waypoint
     * @param y The y coordinate of the waypoint
     */
    public Waypoint(double x, double y) {
        this(new Point(x, y), 0);
    }

    /**
     * Constructs a Waypoint using a Point
     *
     * @param point The point of the Waypoint
     */
    public Waypoint(Point point) {
        this(point.x, point.y);
    }


    /**
     * @return the waypoint as a Point
     */
    public Point getPoint() {
        return point;
    }

    /**
     * @return the x coordinate of the waypoint
     */
    public double getX() {
        return point.getX();
    }

    /**
     * @return the y coordinate of the waypoint
     */
    public double getY() {
        return point.getY();
    }

    /**
     * @return the heading of the waypoint (radians)
     */
    public double getHeading() {
        return this.heading;
    }

    @Override
    public String toString() {
        return "x: " + getX() + " y: " + getY() + " h: " + Math.toDegrees(getHeading());
    }

    /**
     * Adds two waypoints together
     *
     * @param other the other waypoint
     * @return the sum of the two waypoints
     */
    public Waypoint add(Waypoint other) {
        return new Waypoint(point.add(other.getPoint()), heading + other.getHeading());
    }

    /**
     * Returns a new Waypoint that is the sum of the current Waypoint and the given x and y values
     *
     * @param x the x value to add
     * @param y the y value to add
     * @param heading the heading to add
     * @return the sum of the current Waypoint and the given x and y values
     */
    public Waypoint add(double x, double y, double heading) {
        return new Waypoint(point.add(new Point(x, y)), this.heading + heading);
    }
}
