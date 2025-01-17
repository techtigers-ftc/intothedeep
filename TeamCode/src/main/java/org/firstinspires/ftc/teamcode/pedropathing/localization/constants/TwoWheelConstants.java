package org.firstinspires.ftc.teamcode.pedropathing.localization.constants;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Encoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * This is the TwoWheelConstants class. It holds many constants and parameters for the Two Wheel Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

public class TwoWheelConstants {

    /** The number of inches per tick of the encoder for forward movement
     * @value Default Value: .001989436789 */
    public static double forwardTicksToInches = .001989436789;

    /** The number of inches per tick of the encoder for lateral movement (strafing)
     * @value Default Value: .001989436789 */
    public static double strafeTicksToInches = .001989436789;

    /** The y offset of the forward encoder (Deadwheel) from the center of the robot
     * @value Default Value: 1 */
    public static double forwardY = 1;

    /** The x offset of the strafe encoder (Deadwheel) from the center of the robot
     * @value Default Value: -2.5 */
    public static double strafeX = -2.5;

    /** The Hardware Map Name of the IMU (built-in IMU will be Port 0, "imu")
     * @value Default Value: "imu" */
    public static String IMU_HardwareMapName = "imu";

    /** The Hardware Map Name of the Forward Encoder (name of the motor port it is plugged into)
     * @value Default Value: "leftFront" */
    public static String forwardEncoder_HardwareMapName = "leftFront";

    /** The Hardware Map Name of the Strafe Encoder (name of the motor port it is plugged into)
     * @value Default Value: "rightRear" */
    public static String strafeEncoder_HardwareMapName = "rightRear";

    /** The Orientation of the IMU on the robot
     * @value Default Value: new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT) */
    public static RevHubOrientationOnRobot IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    /** The direction of the forward encoder
     * @value Default Value: Encoder.REVERSE */
    public static double forwardEncoderDirection = Encoder.REVERSE;

    /** The direction of the strafe encoder
     * @value Default Value: Encoder.FORWARD */
    public static double strafeEncoderDirection = Encoder.FORWARD;
}
