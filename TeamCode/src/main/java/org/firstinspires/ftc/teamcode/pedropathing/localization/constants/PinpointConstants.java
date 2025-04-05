package org.firstinspires.ftc.teamcode.pedropathing.localization.constants;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedropathing.localization.GoBildaPinpointDriver;

/**
 * This is the PinpointConstants class. It holds many constants and parameters for the Pinpoint Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

public class PinpointConstants {

    /** The Y Offset of the Forward Encoder (Deadwheel) from the center of the robot in DistanceUnit
     * @see #distanceUnit
     * Default Value: 1 */
    public static double forwardY = 88.9;

    /** The X Offset of the Strafe Encoder (Deadwheel) from the center of the robot in DistanceUnit
     * @see #distanceUnit
     * Default Value: -2.5 */
    public static double strafeX = 30;

    /** The Unit of Distance that the Pinpoint uses to measure distance
     * Default Value: DistanceUnit.INCH */
    public static DistanceUnit distanceUnit = DistanceUnit.MM;

    /** The name of the Pinpoint in the hardware map (name of the I2C port it is plugged into)
     * Default Value: "pinpoint" */
    public static String hardwareMapName = "odo";

    /** Use custom yaw scalar for the Pinpoint (overrides the calibration of the Pinpoint
     * Default Value: false */
    public static boolean useYawScalar = false;

    /** Custom Yaw Scalar for the Pinpoint (overrides the calibration of the Pinpoint
     * @see #useYawScalar
     * Default Value: 1.0 */
    public static double yawScalar = 1.0;

    /** Use custom encoder resolution for the Pinpoint
     * Default Value: false */
    public static boolean useCustomEncoderResolution = false;

    /** The Encoder Resolution for the Pinpoint. Used if useCustomEncoderResolution is false
     * @see #useCustomEncoderResolution
     * Default Value: GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD */
    public static GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /** The Encoder Resolution for the Pinpoint. Only used if useCustomEncoderResolution is true
     * @see #useCustomEncoderResolution
     * Default Value: 13.26291192 */
    public static double customEncoderResolution = 13.26291192;

    /** The Encoder Direction for the Forward Encoder (Deadwheel)
     * Default Value: GoBildaPinpointDriver.EncoderDirection.REVERSED */
    public static GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    /** The Encoder Direction for the Strafe Encoder (Deadwheel)
     * Default Value: GoBildaPinpointDriver.EncoderDirection.FORWARD */
    public static GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
}
