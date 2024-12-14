package org.firstinspires.ftc.teamcode.cv;

import org.opencv.core.Point;

/**
 * A class used to represent the location of sample objects within the camera view
 */
public class Sample {
    private double x;
    private double y;
    private double orientation;
    private Point[] centerLine;

    /**
     * Creates a new Sample object
     * @param x the x position of the sample within the camera view
     * @param y the y position of the sample within the camera view
     * @param orientation the orientation of the sample relative to the camera
     * @paran centerLine an array of the two points which make up the center line of the block
     */
    public Sample(double x, double y, double orientation, Point[] centerLine) {
        this.x = x;
        this.y = y;
        this.orientation = orientation;
        this.centerLine = centerLine;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getOrientation() {
        return orientation;
    }

    public void setOrientation(double orientation) {
        this.orientation = orientation;
    }

    public Point[] getCenterLine() {
        return centerLine;
    }

    public void setCenterLine(Point[] centerLine) {
        this.centerLine = centerLine;
    }
}
