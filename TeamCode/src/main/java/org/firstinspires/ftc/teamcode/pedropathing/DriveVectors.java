package org.firstinspires.ftc.teamcode.pedropathing;

import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.Vector;

/**
 * Class for holding the pedro pathing drive vectors to be used to drive the motors
 */
public class DriveVectors {
    public final Vector correctivePower;
    public final Vector headingPower;
    public final Vector pathingPower;
    public final double robotHeading;

    /**
     * Constructs a new DriveVectors object
     *
     * @param correctivePower the corrective power vector
     * @param headingPower    the heading power vector
     * @param pathingPower    the pathing power vector
     * @param robotHeading    the robot's heading
     */
    public DriveVectors(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        this.correctivePower = correctivePower;
        this.headingPower = headingPower;
        this.pathingPower = pathingPower;
        this.robotHeading = robotHeading;
    }
}
