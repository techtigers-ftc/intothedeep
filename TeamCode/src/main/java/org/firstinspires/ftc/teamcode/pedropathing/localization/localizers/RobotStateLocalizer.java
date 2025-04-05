package org.firstinspires.ftc.teamcode.pedropathing.localization.localizers;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.MathFunctions;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Vector;
import org.firstinspires.ftc.teamcode.utils.PoseTranslator;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.paths.Waypoint;

/**
 * A localizer that uses the RobotState to localize the robot.
 */
public class RobotStateLocalizer extends Localizer {
    private RobotState robotState;
    private Pose currentPose;
    private double previousHeading;
    private double totalHeading;
    private Pose currentVelocity;

    /**
     * Constructs a new RobotStateLocalizer.
     *
     * @param robotState The robot state
     */
    public RobotStateLocalizer(RobotState robotState) {
        super();
        this.robotState = robotState;
        previousHeading = 0;
        totalHeading = 0;
        currentVelocity = new Pose(0, 0);
    }

    @Override
    public Pose getPose() {
        return currentPose;
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        // Intentionally not doing this, setStartPose should be in odometry
        // subsystem
    }

    @Override
    public void setPose(Pose setPose) {
        // Intentionally not doing this, pose shouldn't be overridden
    }

    /**
     * This updates the total heading of the robot. The Pinpoint handles all other updates itself.
     */
    @Override
    public void update() {
        currentPose =
                PoseTranslator.waypointToPedroPose(robotState.getRobotCurrentPose());
        totalHeading += MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), previousHeading);
        previousHeading = currentPose.getHeading();
        currentVelocity =
                PoseTranslator.waypointToPose(robotState.getRobotVelocity());
    }

    /**
     * This updates the total heading of the robot. The Pinpoint handles all other updates itself.
     */
    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        // Intentionally not doing this, forward multiplier isn't used
        return 0;
    }

    @Override
    public double getLateralMultiplier() {
        // Intentionally not doing this, forward multiplier isn't used
        return 0;
    }

    @Override
    public double getTurningMultiplier() {
        // Intentionally not doing this, forward multiplier isn't used
        return 0;
    }

    @Override
    public void resetIMU() throws InterruptedException {
        // Intentionally not doing this, IMU should be reset in odometry subsystem
    }

    @Override
    public boolean isNAN() {
        Waypoint robotPose = robotState.getRobotCurrentPose();
        return Double.isNaN(robotPose.getX()) || Double.isNaN(robotPose.getY()) || Double.isNaN(robotPose.getHeading());
    }
}
