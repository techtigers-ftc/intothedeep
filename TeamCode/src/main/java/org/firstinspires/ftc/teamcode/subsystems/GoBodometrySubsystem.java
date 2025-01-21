package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import java.util.function.DoubleSupplier;

import team.techtigers.base.CloseableSubsystem;
import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.paths.geometry.Point;
import team.techtigers.core.paths.geometry.Rectangle;

/**
 * The odometry subsystem, using the localizer from Roadrunner.
 */
public class GoBodometrySubsystem extends CloseableSubsystem {
    private final GoBildaPinpointDriver odo;
    private final RobotState robotState;
    private Waypoint startPose;

    //Defines the points and rectangle for the ascent zone submersible zone
    private static final Point ASCENT_SUBMERSIBLE_TR = new Point(56, 47);
    private static final Point ASCENT_SUBMERSIBLE_BL = new Point(47, 93);
    private static final Rectangle ASCENT_SUBMERSIBLE_ZONE = new Rectangle(ASCENT_SUBMERSIBLE_TR, ASCENT_SUBMERSIBLE_BL);

    // Defines the points and rectangle for the chamber zone submersible zone
    private static final Point CHAMBER_SUBMERSIBLE_TR = new Point(84, 47);
    private static final Point CHAMBER_SUBMERSIBLE_BL = new Point(58, 32);
    private static final Rectangle CHAMBER_SUBMERSIBLE_ZONE = new Rectangle(CHAMBER_SUBMERSIBLE_TR, CHAMBER_SUBMERSIBLE_BL);


    /**
     * Initializes a new RoadometrySubsystem.
     *
     * @param hardwareMap The hardware map, used to get hardware references
     * @param robotState       The state into which localization values are
     *                      written.
     * @param startPose   The starting pose of the robot.
     */
    public GoBodometrySubsystem(HardwareMap hardwareMap, RobotState robotState,
                                Waypoint startPose) {
        this.robotState = robotState;
        this.startPose = startPose;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(30, 88.9);

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

//        startPose = new Waypoint(startPose.getX()*25.4, startPose.getY()*25.4, startPose.getHeading());
//        odo.setPosition(startPose);
    }

    /**
     * Initializes a new RoadometrySubsystem with the start pose at (0, 0, 0).
     *
     * @param hardwareMap The hardware map, used to get hardware references
     * @param state       The state into which localization values are written.
     */
    public GoBodometrySubsystem(HardwareMap hardwareMap, RobotState state) {
        this(hardwareMap, state, new Waypoint(0, 0));
    }

    private Waypoint rotateWaypoint(Waypoint waypoint, double heading) {
        double x = waypoint.getX();
        double y = waypoint.getY();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return new Waypoint(x * cos - y * sin, x * sin + y * cos, waypoint.getHeading());
    }

    private DoubleSupplier getVisionIntakeHeading(Waypoint robotCurrentPose) {
        Point robotCurrentPoint = new Point(robotCurrentPose.getX(), robotCurrentPose.getY());
        if(ASCENT_SUBMERSIBLE_ZONE.contains(robotCurrentPoint)) {
            return () -> Math.toRadians(0);
        } else if(CHAMBER_SUBMERSIBLE_ZONE.contains(robotCurrentPoint)) {
            return () -> Math.toRadians(0);
        } else{
            return robotCurrentPose::getHeading;
        }
    }

    @Override
    public void periodic() {
        odo.update();

        double heading = odo.getHeading();
        double headingVelocity = odo.getHeadingVelocity();

        Waypoint robotPose = new Waypoint(odo.getPosX()/25.4,
                odo.getPosY()/25.4, heading);
        robotPose = rotateWaypoint(robotPose, startPose.getHeading());
        robotPose = new Waypoint(robotPose.getX() + startPose.getX(),
                robotPose.getY() + startPose.getY(),
                robotPose.getHeading() + startPose.getHeading());

        Waypoint robotVelocity = new Waypoint(odo.getVelX()/25.4,
                odo.getVelY()/25.4, headingVelocity);
        robotVelocity = rotateWaypoint(robotVelocity, startPose.getHeading());

        robotState.setVisionIntakeHeadingSupplier(getVisionIntakeHeading(robotPose));
        robotState.setRobotPose(robotPose);
        robotState.setRobotVelocity(robotVelocity);
    }
}