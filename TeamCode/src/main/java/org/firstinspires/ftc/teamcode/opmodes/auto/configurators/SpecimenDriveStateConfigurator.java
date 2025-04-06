package org.firstinspires.ftc.teamcode.opmodes.auto.configurators;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveFromBasketToSpecimenPark;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveFromChamberSampleDropState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstPushState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstSpecimenIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenDropState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToSpecimenPark;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPoseState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropSpecimenState;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;

/**
 * A class used to configure Specimen Drive States.
 */
@Config
public class SpecimenDriveStateConfigurator {
    public static final double HUMUNGOUS_TOLERANCE = 8;
    public static final double LARGE_TOLERANCE = 5;
    public static final double LARGE_ANGLE_TOLERANCE = Math.toRadians(5);
    public static final double MEDIUM_TOLERANCE = 4;
    public static final double SMALL_TOLERANCE = 3;
    public static final double SMALL_ANGLE_TOLERANCE = Math.toRadians(3);
    public static final double MINISCULE_TOLERANCE = 1.25;
    public static final double MINISCULE_ANGLE_TOLERANCE = Math.toRadians(1.5);
    public static final double MICROSCOPIC_TOLERANCE = 1;

    /**
     * Configures the DriveToPreloadDropState.
     *
     * @param state The DriveToPreloadDropState to configure
     */
    public static void configPreloadDrop(DriveToPreloadDropSpecimenState state) {
        state.setTranslationalPIDF(0.08, 0, 0.001, 0);
        state.setDrivePIDF(0.004, 0, 0.0019, 0.6, 0);
        state.setHeadingPIDF(0.9, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(79.25, 7.25),
                                        new Point(72, 42)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(MEDIUM_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive to the first specimen intake from the wall, dropping the sample on the way
     *
     * @param state the state to configure
     */
    public static void configFirstIntake(DriveToFirstSpecimenIntakeState state) {
        state.setTranslationalPIDF(0.03, 0, 0.001, 0);
        state.setDrivePIDF(0.008, 0, 0.005, 0.6, 0);
        state.setHeadingPIDF(0.5, 0, 0.03, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierCurve(
                                new Point(72, 42),
                                new Point(113, 12)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive to the first specimen drop
     *
     * @param state the state to configure
     */
    public static void configFirstDrop(DriveToGeneralSpecimenDropState state) {
        state.setTranslationalPIDF(0.08, 0, 0.004, 0);
        state.setDrivePIDF(0.007, 0, 0.004, 0.6, 0);
        state.setHeadingPIDF(0.9, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(113, 12),
                                new Point(74, 42)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the first intake state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configDriveToFirstPush(DriveToFirstPushState state) {
        state.setTranslationalPIDF(0.06, 0, 0.001, 0);
        state.setDrivePIDF(0.004, 0, 0.0019, 0.6, 0);
        state.setHeadingPIDF(1.1, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierCurve(
                                new Point(72, 42),
                                new Point(77.25, 32),
                                new Point(123, 28),
                                new Point(100, 55),
                                new Point(117.25, 52)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive state with a path chain to push the first sample into the observation zone
     *
     * @param state the state to configure
     */
    public static void configFirstPush(DriveToPoseState state) {
        state.setTranslationalPIDF(0.06, 0, 0.001, 0);
        state.setDrivePIDF(0.01, 0, 0.0019, 0.6, 0);
        state.setHeadingPIDF(0.7, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        // Push first sample
                        .addBezierLine(
                                new Point(117.25, 52),
                                new Point(117.25, 22)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive state with a path to get to the spot for the second intake
     *
     * @param state the state to configure
     */
    public static void configDriveToSecondPush(DriveToPoseState state) {
        state.setTranslationalPIDF(0.06, 0, 0.001, 0);
        state.setDrivePIDF(0.01, 0, 0.0019, 0.6, 0);
        state.setHeadingPIDF(0.7, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        // Curve to second sample
                        .addBezierCurve(
                                new Point(119.25, 22),
                                new Point(110, 52),
                                new Point(126, 48)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))

                        .build()
        );

        state.setTolerance(HUMUNGOUS_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive state with a path to push the second sample into the observation zone
     *
     * @param state the state to configure
     */
    public static void configSecondPush(DriveToPoseState state) {
        state.setTranslationalPIDF(0.06, 0, 0.001, 0);
        state.setDrivePIDF(0.01, 0, 0.0019, 0.6, 0);
        state.setHeadingPIDF(0.7, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        // Push second sample
                        .addBezierLine(
                                new Point(126, 48),
                                new Point(126, 22)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))

                        .build()
        );

        state.setTolerance(HUMUNGOUS_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive state with a path to go to the third intake spot
     *
     * @param state the state to configure
     */
    public static void configDriveToThirdPush(DriveToPoseState state) {
        state.setTranslationalPIDF(0.06, 0, 0.001, 0);
        state.setDrivePIDF(0.01, 0, 0.0019, 0.6, 0);
        state.setHeadingPIDF(0.7, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        // Curve to third sample
                        .addBezierCurve(
                                new Point(126, 22),
                                new Point(116, 52),
                                new Point(136, 50)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(HUMUNGOUS_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive state with a path to push the third sample into the observation zone
     *
     * @param state the state to configure
     */
    public static void configThirdPush(DriveToPoseState state) {
        state.setTranslationalPIDF(0.06, 0, 0.001, 0);
        state.setDrivePIDF(0.01, 0, 0.0019, 0.6, 0);
        state.setHeadingPIDF(0.7, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        // Push third sample
                        .addBezierLine(
                                new Point(136, 48),
                                new Point(136, 25)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive to the second specimen intake from the wall
     *
     * @param state the state to configure
     */
    public static void configSecondIntake(DriveToPoseState state) {
        state.setTranslationalPIDF(0.06, 0, 0.001, 0);
        state.setDrivePIDF(0.003, 0, 0.0019, 0.6, 0);
        state.setHeadingPIDF(0.7, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                // 100 is correct for the y, don't change it
                                new Point(135, 100),
                                new Point(129, 15)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive to the second specimen drop
     *
     * @param state the state to configure
     */
    public static void configSecondDrop(DriveToGeneralSpecimenDropState state) {
        state.setTranslationalPIDF(0.08, 0, 0.004, 0);
        state.setDrivePIDF(0.0055, 0, 0.0035, 0.6, 0);
        state.setHeadingPIDF(0.9, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(129, 9),
                                new Point(71.5, 42)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive to the general specimen intake from the wall
     *
     * @param state the state to configure
     */
    public static void configGeneralIntake(DriveStateBase state) {
        state.setTranslationalPIDF(0.03, 0, 0.001, 0);
        state.setDrivePIDF(0.008, 0, 0.0045, 0.6, 0);
        state.setHeadingPIDF(0.5, 0, 0.03, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierCurve(
                                new Point(71, 42),
                                new Point(113, 13.5)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive to the general specimen drop
     *
     * @param state the state to configure
     */
    public static void configGeneralDrop(DriveStateBase state) {
        state.setTranslationalPIDF(0.08, 0, 0.004, 0);
        state.setDrivePIDF(0.007, 0, 0.004, 0.6, 0);
        state.setHeadingPIDF(0.9, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierCurve(
                                new Point(113, 13.5),
                                new Point(72, 42)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive to park state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configDriveFromChamberPark(DriveToSpecimenPark state) {
        state.setTranslationalPIDF(0.08, 0, 0.001, 0);
        state.setDrivePIDF(0.0055, 0, 0.0035, 0.6, 0);
        state.setHeadingPIDF(0.9, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(72, 42),
                                new Point(120, 15)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive to park state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configDriveFromBasketPark(DriveFromBasketToSpecimenPark state) {
        state.setTranslationalPIDF(0.15, 0, 0, 0);
        state.setHeadingPIDF(1, 0, 0, 0);
        state.setDrivePIDF(0.012, 0, 0.003, 0.6, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(12, 10),
                                new Point(100, 15)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the Sample Drop.
     *
     * @param state The DriveToGeneralSampleDropState to configure
     */
    public static void configSampleDrop(DriveFromChamberSampleDropState state) {
        state.setTranslationalPIDF(0.15, 0, 0, 0);
        state.setHeadingPIDF(1, 0, 0, 0);
        state.setDrivePIDF(0.012, 0, 0.004, 0.6, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(75, 41.5),
                                        new Point(60, 35),
                                        new Point(12, 10)
                                )
                        )
//                        .setConstantHeadingInterpolation(Math.toRadians(30))
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(30))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }
}
