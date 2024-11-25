package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Arrays;

/**
 * Constants for the robot, such as physical dimensions and motor constraints.
 **/
@Config
public class DriveConstants {
    /*
     * These are motor constants that should be listed online for your motors.
     *
     * Values from online
     */
    public static final double TICKS_PER_REV = 2000;
    public static final double MAX_RPM = 512;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     *
     * RUN_USING_ENCODER mode
     */
    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     *
     * Physical Constants, as measured by the team
     */
    public static double WHEEL_RADIUS = 1.2; // inches
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 10.25; // in 10.25
    public static double WHEEL_BASE = 7.75; // inches

    public static double xInPerTick = 1 / 338.56;
    public static double yInPerTick = 1 / 337.71;

    // Lateral Multiplier is a constant used to account for y encoder drift. It is defaulted to 1.0
    public static double LATERAL_MULTIPLIER = 1.0;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     *
     * Feedforward Constants
     */
//    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kV = 0.01055; // 0.0085
    public static double kA = 0.003; // 0.0035
    public static double kStatic = 0.04; // 0.11

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * IRunner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     *
     * Max Velocity and Acceleration
     */
    public static double MAX_VEL = 60;
    public static double MAX_ACCEL = 60;
    public static double MAX_ANG_VEL = Math.toRadians(250);
    /*
     * Velocity and Acceleration Constraints for the Mecanum Drive
     */
    public static double MAX_ANG_ACCEL = Math.toRadians(60);
    /*
     * Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
     */
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static final double FIELD_WIDTH = 141.25;

    /**
     * Converts encoder ticks to inches in the x direction
     *
     * @param ticks The encoder ticks
     * @return The number of inches
     */
    public static double encoderTicksToInchesVertical(double ticks) {
        return ticks * xInPerTick;
    }

    /**
     * Converts encoder ticks to inches in the y direction
     *
     * @param ticks The encoder ticks
     * @return The number of inches
     */
    public static double encoderTicksToInchesHorizontal(double ticks) {
        return ticks * yInPerTick;
    }

    /**
     * Converts RPM to velocity
     *
     * @param rpm The RPM
     * @return The velocity in inches per second
     */
    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    /**
     * Returns the motor velocity
     *
     * @param ticksPerSecond The number of ticks per second
     * @return The motor velocity
     */
    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
