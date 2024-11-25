package team.techtigers.core.paths.geometry;

import java.util.Arrays;

/**
 * A rectangle
 */
public class Rectangle {
    /**
     * Top right, bottom left, and center
     */
    private Point topRight;
    private Point bottomLeft;
    public Point center;


    /**
     * Construct a Rectangle
     *
     * @param topRight   The top right Point
     * @param bottomLeft The top left Point
     */
    public Rectangle(Point topRight, Point bottomLeft) {
        this.topRight = topRight;
        this.bottomLeft = bottomLeft;
        center = this.topRight.add(this.bottomLeft).multiply(0.5);
    }

    /**
     * @param point A Point
     * @return Whether this Rectangle contains the Point
     */
    public boolean contains(Point point) {
        return point.x > bottomLeft.x
                && point.x < topRight.x
                && point.y > bottomLeft.y
                && point.y < topRight.y;
    }

    /**
     * @param other Another Rectangle
     * @return Whether this Rectangle overlaps with another Rectangle
     */
    public boolean overlaps(Rectangle other) {
        return Arrays.stream(other.getPoints()).sequential().anyMatch(this::contains);
    }

    /**
     * @return The points of this Rectangle
     */
    public Point[] getPoints() {
        return new Point[]{topRight, bottomLeft, new Point(topRight.x, bottomLeft.y), new Point(topRight.y, bottomLeft.x)};
    }

    @Override
    public String toString() {
        return this.center.toString();
    }

    /**
     * Scale a rectangle large by scaling each of its points
     *
     * @param factor How much bigger the rectangle will get
     */
    public void scale(double factor) {
        topRight = topRight.multiply(factor);
        bottomLeft = bottomLeft.multiply(factor);
        center = center.multiply(factor);
    }

}
