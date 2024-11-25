package team.techtigers.core.paths.geometry;

/**
 * A two dimensional point
 */
public class Point {
    /**
     * The x and y coordinates
     */
    public final double x, y;

    /**
     * Construct a Point
     *
     * @param x The x coordinate
     * @param y The y coordinate
     */
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * @return Get the x value
     */
    public double getX() {
        return this.x;
    }

    /**
     * @return Get the y value
     */
    public double getY() {
        return this.y;
    }


    @Override
    public String toString() {
        return "Point(" + x + ", " + y + ")";
    }

    /**
     * @return The angle the point makes with the origin
     */
    public double polarAngle() {
        return Math.atan(this.y / this.x);
    }

    /**
     * @return The point's distance to the origin
     */
    public double magnitude() {
        return Math.sqrt(this.y * this.y + this.x * this.x);
    }

    /**
     * Perform vector addition between two Points
     *
     * @param other The other Point
     * @return The resulting Point
     */
    public Point add(Point other) {
        return new Point(this.x + other.x, this.y + other.y);
    }

    /**
     * Perform vector subtraction between two Points
     *
     * @param other The other Point
     * @return The resulting Point
     */
    public Point minus(Point other) {
        return new Point(this.x - other.x, this.y - other.y);
    }

    /**
     * Multiply a Point by a scalar (scalar multiplication)
     *
     * @param factor The multiplication factor
     * @return The resulting Point
     */
    public Point multiply(double factor) {
        return new Point(this.x * factor, this.y * factor);
    }


    /**
     * Rotate a Point about the origin
     *
     * @param angle The angle the Point will be rotated counterclockwise
     * @return The resulting Point
     */
    public Point transform(double angle) {
        return new Point(this.magnitude() * (Math.cos(this.polarAngle() + angle)), this.magnitude() * (Math.sin(this.polarAngle() + angle)));
    }

    /**
     * @param other The other Point
     * @return The distance between the two Points
     */
    public double dist(Point other) {
        return this.add(other.multiply(-1)).magnitude();
    }

    /**
     * @param other Another Point
     * @return The angle between two Points
     */
    public double angleTo(Point other) {
        return Math.acos((x * other.x + y * other.y) / (magnitude() * other.magnitude()));
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Point
                && ((Point) other).x == x
                && ((Point) other).y == y;
    }

    @Override
    public int hashCode() {
        return (int) (x + 31 * y);
    }

    /**
     * Find the Manhattan distance from any point to another
     *
     * @param other
     * @return the Manhattan distance
     */
    public double manhattan(Point other) {
        return Math.abs(other.x - this.x) + Math.abs(other.y - this.y);
    }
}
