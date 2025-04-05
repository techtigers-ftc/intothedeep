package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.CloseableSubsystem;
import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.paths.geometry.Point;
import team.techtigers.core.paths.geometry.Rectangle;

/**
 * The odometry subsystem, using the localizer from Roadrunner.
 */
public class GoBodometrySubsystem extends CloseableSubsystem {
    //Defines the points and rectangle for the ascent zone submersible zone on red side
    private static final Point ASCENT_SUBMERSIBLE_TR_ALLIANCE = new Point(54,
            90);
    private static final Point ASCENT_SUBMERSIBLE_BL_ALLIANCE = new Point(36,
            52);
    private static final Rectangle ASCENT_SUBMERSIBLE_ZONE_ALLIANCE = new Rectangle(ASCENT_SUBMERSIBLE_TR_ALLIANCE, ASCENT_SUBMERSIBLE_BL_ALLIANCE);
    private static final double ASCENT_SUBMERSIBLE_ZONE_ALLIANCE_HEADING = Math.toRadians(0);
    // Defines the points and rectangle for the chamber zone submersible zone on red side
    private static final Point CHAMBER_SUBMERSIBLE_TR_ALLIANCE = new Point(83,
            46);
    private static final Point CHAMBER_SUBMERSIBLE_BL_ALLIANCE = new Point(59,
            34);
    private static final Rectangle CHAMBER_SUBMERSIBLE_ZONE_ALLIANCE = new Rectangle(CHAMBER_SUBMERSIBLE_TR_ALLIANCE, CHAMBER_SUBMERSIBLE_BL_ALLIANCE);
    private static final double CHAMBER_SUBMERSIBLE_ZONE_ALLIANCE_HEADING = Math.toRadians(90);
    //Defines the points and rectangle for the ascent zone submersible zone on blue side
    private static final Point ASCENT_SUBMERSIBLE_TR_OPPONENT = new Point(106
            , 91);
    private static final Point ASCENT_SUBMERSIBLE_BL_OPPONENT = new Point(88,
            52);
    private static final Rectangle ASCENT_SUBMERSIBLE_ZONE_OPPONENT = new Rectangle(ASCENT_SUBMERSIBLE_TR_OPPONENT, ASCENT_SUBMERSIBLE_BL_OPPONENT);
    private static final double ASCENT_SUBMERSIBLE_ZONE_OPPONENT_HEADING = Math.toRadians(180);
    // Defines the points and rectangle for the chamber zone submersible zone on blue side
    private static final Point CHAMBER_SUBMERSIBLE_TR_OPPONENT = new Point(83,
            108);
    private static final Point CHAMBER_SUBMERSIBLE_BL_OPPONENT = new Point(59,
            96);
    private static final Rectangle CHAMBER_SUBMERSIBLE_ZONE_OPPONENT = new Rectangle(CHAMBER_SUBMERSIBLE_TR_OPPONENT, CHAMBER_SUBMERSIBLE_BL_OPPONENT);
    private static final double CHAMBER_SUBMERSIBLE_ZONE_OPPONENT_HEADING = Math.toRadians(270);
    private final GoBildaPinpointDriver odo;
    private final RobotState robotState;
    private Waypoint startPose;


    /**
     * Initializes a new RoadometrySubsystem.
     *
     * @param hardwareMap The hardware map, used to get hardware references
     * @param robotState  The state into which localization values are
     *                    written.
     * @param startPose   The starting pose of the robot.
     */
    public GoBodometrySubsystem(HardwareMap hardwareMap, RobotState robotState,
                                Waypoint startPose) {
        this.robotState = robotState;
        this.startPose = startPose;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

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

    private double getVisionIntakeHeading(Waypoint robotCurrentPose) {
        Point robotCurrentPoint = new Point(robotCurrentPose.getX(), robotCurrentPose.getY());
        if (ASCENT_SUBMERSIBLE_ZONE_ALLIANCE.contains(robotCurrentPoint)) {
            return ASCENT_SUBMERSIBLE_ZONE_ALLIANCE_HEADING;
        } else if (CHAMBER_SUBMERSIBLE_ZONE_ALLIANCE.contains(robotCurrentPoint)) {
            return CHAMBER_SUBMERSIBLE_ZONE_ALLIANCE_HEADING;
        } else if (ASCENT_SUBMERSIBLE_ZONE_OPPONENT.contains(robotCurrentPoint)) {
            return ASCENT_SUBMERSIBLE_ZONE_OPPONENT_HEADING;
        } else if (CHAMBER_SUBMERSIBLE_ZONE_OPPONENT.contains(robotCurrentPoint)) {
            return CHAMBER_SUBMERSIBLE_ZONE_OPPONENT_HEADING;
        } else {
            return robotCurrentPose.getHeading();
        }
    }

    @Override
    public void periodic() {
        odo.update();

        double heading = odo.getHeading();
        double headingVelocity = odo.getHeadingVelocity();

        Waypoint robotPose = new Waypoint(odo.getPosX() / 25.4,
                odo.getPosY() / 25.4, heading);

        RobotLog.dd(tag, "Odo X: %f, Odo Y: %f, H: %f", robotPose.getX(), robotPose.getY(), robotPose.getHeading());

        robotPose = rotateWaypoint(robotPose, startPose.getHeading());
        robotPose = new Waypoint(robotPose.getX() + startPose.getX(),
                robotPose.getY() + startPose.getY(),
                (robotPose.getHeading() + startPose.getHeading()) % (2 * Math.PI));

        if (Double.isNaN(robotPose.getX()) || Double.isNaN(robotPose.getY()) || Double.isNaN(robotPose.getHeading())) {
            RobotLog.ww(tag, "Pose is dead");
            return;
        }
        robotState.setRobotPose(robotPose);

        Waypoint robotVelocity = new Waypoint(odo.getVelX() / 25.4,
                odo.getVelY() / 25.4, headingVelocity);
        RobotLog.dd(tag, "Odo Vel X: %f, Odo Vel Y: %f, Odo Vel H: %f", robotVelocity.getX(), robotVelocity.getY(), robotVelocity.getHeading());
        robotVelocity = rotateWaypoint(robotVelocity, startPose.getHeading());

        if (Double.isNaN(robotVelocity.getX()) || Double.isNaN(robotVelocity.getY()) || Double.isNaN(robotVelocity.getHeading())) {
            RobotLog.ww(tag, "Velocity is dead");
            return;
        }
        robotState.setRobotVelocity(robotVelocity);

//        robotState.setVisionIntakeHeading(getVisionIntakeHeading(robotPose));
    }

    public void setPose(Waypoint waypoint) {
        startPose = startPose.add(waypoint);
        startPose = new Waypoint(startPose.getX(), startPose.getY(), robotState.getRobotCurrentPose().getHeading() - waypoint.getHeading());
    }
}