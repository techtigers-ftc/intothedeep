package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.CloseableSubsytem;
import team.techtigers.core.paths.Waypoint;

/**
 * The odometry subsystem, using the localizer from Roadrunner.
 */
public class GoBodometrySubsystem extends CloseableSubsytem {
    private final GoBildaPinpointDriver odo;
    private final RobotState robotState;

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

        odo.setPosition(startPose);
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

    @Override
    public void periodic() {
        odo.update();

        double heading = odo.getHeading();
        double headingVelocity = odo.getHeadingVelocity();

        robotState.setRobotPose(new Waypoint(odo.getPosX()/25.4,
                odo.getPosY()/25.4, heading));
        robotState.setRobotVelocity(new Waypoint(odo.getVelX()/25.4,
                odo.getVelY()/25.4, headingVelocity));
    }
}