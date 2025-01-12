package org.firstinspires.ftc.teamcode.pedropathing.localization.constants;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Encoder;

/**
 * This is the DriveEncoderConstants class. It holds many constants and parameters for the Drive Encoder Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

public class DriveEncoderConstants {

    /** The number of inches per ticks of the encoder for forward movement
     * @value Default Value: 1 */
    public static double forwardTicksToInches = 1;

    /** The number of inches per ticks of the encoder for lateral movement (strafing)
     * @value Default Value: 1 */
    public static double strafeTicksToInches = 1;

    /** The number of inches per ticks of the encoder for turning
     * @value Default Value: 1 */
    public static double turnTicksToInches = 1;

    public static double robot_Width = 1;
    public static double robot_Length = 1;

    /** The direction of the left front encoder
     * @value Default Value: Encoder.REVERSE */
    public static double leftFrontEncoderDirection = Encoder.REVERSE;

    /** The direction of the right front encoder
     * @value Default Value: Encoder.FORWARD */
    public static double rightFrontEncoderDirection = Encoder.FORWARD;

    /** The direction of the left rear encoder
     * @value Default Value: Encoder.REVERSE */
    public static double leftRearEncoderDirection = Encoder.REVERSE;

    /** The direction of the right rear encoder
     * @value Default Value: Encoder.FORWARD */
    public static double rightRearEncoderDirection = Encoder.FORWARD;
}
