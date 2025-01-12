package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

/**
 * These constants can be used to tune values using FTC dashboard.
 */
@Config
public class TuningConstants {
    public static double translationalP = 0.15;
    public static double translationalI = 0;
    public static double translationalD = 0.01;
    public static double driveP = 0.005;
    public static double driveI = 0;
    public static double driveD = 0.0001;
    public static double driveT = 0.6;
    public static double headingP = 1;
    public static double headingI = 0;
    public static double headingD = 0.03;
    public static double headingF = 0;
    public static double translationalF = 0;
    public static double driveF = 0;
//
//    /** Translational PIDF coefficients (don't use integral)
//     * @value Default Value: new CustomPIDFCoefficients(0.1,0,0,0); */
//    public static CustomPIDFCoefficients translationalPIDFCoefficients = new CustomPIDFCoefficients(
//            0.15,
//            0,
//            0.01,
//            0);
//
//    /** Translational Integral
//     * @value Default Value: new CustomPIDFCoefficients(0,0,0,0); */
//    public static CustomPIDFCoefficients translationalIntegral = new CustomPIDFCoefficients(
//            0,
//            0,
//            0,
//            0);
//
//    /** Feed forward constant added on to the translational PIDF
//     * @value Default Value: 0.015 */
//    public static double translationalPIDFFeedForward = 0.015;
//
//
//    /** Heading error PIDF coefficients
//     * @value Default Value: new CustomPIDFCoefficients(1,0,0,0); */
//    public static CustomPIDFCoefficients headingPIDFCoefficients = new CustomPIDFCoefficients(
//            1,
//            0,
//            0.03,
//            0);
//
//    /** Feed forward constant added on to the heading PIDF
//     * @value Default Value: 0.01 */
//    public static double headingPIDFFeedForward = 0.01;
//
//
//    /** Drive PIDF coefficients
//     * @value Default Value: new CustomFilteredPIDFCoefficients(0.025,0,0.00001,0.6,0); */
//    public static CustomFilteredPIDFCoefficients drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(
//            0.005,
//            0,
//            0.0001,
//            0.6,
//            0);
//
//    /** Feed forward constant added on to the drive PIDF
//     * @value Default Value: 0.01 */
//    public static double drivePIDFFeedForward = 0.01;
}
