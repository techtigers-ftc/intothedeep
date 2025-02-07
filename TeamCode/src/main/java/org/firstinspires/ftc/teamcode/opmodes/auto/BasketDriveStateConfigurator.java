package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSampleDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSampleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSubmersibleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToPreloadDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToSubmersible;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;

/**
 * A class used to configure BasketDriveStates.
 */
public class BasketDriveStateConfigurator {
    private static final double LARGE_TOLERANCE = 3;
    private static final double LARGE_ANGLE_TOLERANCE = Math.toRadians(5);
    private static final double SMALL_TOLERANCE = 1.5;
    private static final double SMALL_ANGLE_TOLERANCE = Math.toRadians(3);

    /**
     * Configures the DriveToPreloadDropState
     *
     * @param state The DriveToPreloadDropState to configure
     */
    public static void configPreloadDrop(DriveToPreloadDropState state) {
        state.setTranslationalPIDF(0.1, 0, 0.01, 0);
        state.setSecondaryTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setDrivePIDF(0.003, 0, 0.00035, 0.6, 0);
        state.setSecondaryDrivePIDF(0.003, 0, 0.0002, 0.6, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(29.75, 7.25),
                                        new Point(10, 11)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90),
                                Math.toRadians(45))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the DriveToFirstSampleIntakeState.
     *
     * @param state The DriveToFirstSampleIntakeState to configure
     */
    public static void configFirstSampleIntake(DriveToGeneralSampleIntakeState state) {
        state.setTranslationalPIDF(0.1, 0, 0.01, 0);
        state.setSecondaryTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setDrivePIDF(0.004, 0, 0.00035, 0.6, 0);
        state.setSecondaryDrivePIDF(0.003, 0, 0.0002, 0.6, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(10, 11),
                                        new Point(14, 25)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45),
                                Math.toRadians(68))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(Math.toRadians(2));
    }

    /**
     * Configures the FirstSampleDrop.
     *
     * @param state The FirstSampleDrop to configure
     */
    public static void configFirstSampleDrop(DriveToGeneralSampleDropState state) {
        state.setTranslationalPIDF(0.1, 0, 0.01, 0);
        state.setSecondaryTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setDrivePIDF(0.002, 0, 0.00035, 0.6, 0);
        state.setSecondaryDrivePIDF(0.003, 0, 0.0002, 0.6, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(14, 25),
                                        new Point(10, 11)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(68),
                                Math.toRadians(45))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the SecondSampleIntake.
     *
     * @param state The SecondSampleIntake to configure
     */
    public static void configSecondSampleIntake(DriveToGeneralSampleIntakeState state) {
        state.setTranslationalPIDF(0.1, 0, 0.01, 0);
        state.setSecondaryTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setDrivePIDF(0.004, 0, 0.00035, 0.6, 0);
        state.setSecondaryDrivePIDF(0.003, 0, 0.0002, 0.6, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(10, 11),
                                        new Point(12, 25)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45),
                                Math.toRadians(90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the SecondSampleDrop.
     *
     * @param state The SecondSampleDrop to configure
     */
    public static void configSecondSampleDrop(DriveToGeneralSampleDropState state) {
        state.setTranslationalPIDF(0.1, 0, 0.01, 0);
        state.setSecondaryTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setDrivePIDF(0.002, 0, 0.00035, 0.6, 0);
        state.setSecondaryDrivePIDF(0.003, 0, 0.0002, 0.6, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(12, 25),
                                        new Point(10, 11)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90),
                                Math.toRadians(45))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the ThirdSampleIntake.
     *
     * @param state The ThirdSampleIntake to configure
     */
    public static void configThirdSampleIntake(DriveToGeneralSampleIntakeState state) {
        state.setPIDSToTuning();
        state.setTranslationalPIDF(0.1, 0, 0.01, 0);
        state.setSecondaryTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setHeadingPIDF(0.85, 0, 0.03, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setDrivePIDF(0.004, 0, 0.00035, 0.6, 0);
        state.setSecondaryDrivePIDF(0.003, 0, 0.0002, 0.6, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(10, 11),
                                        new Point(10, 25)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45),
                                Math.toRadians(112))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the ThirdSampleDrop.
     *
     * @param state The ThirdSampleDrop to configure
     */
    public static void configThirdSampleDrop(DriveToGeneralSampleDropState state) {
        state.setTranslationalPIDF(0.1, 0, 0.01, 0);
        state.setSecondaryTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setDrivePIDF(0.002, 0, 0.00035, 0.6, 0);
        state.setSecondaryDrivePIDF(0.003, 0, 0.0002, 0.6, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(14.5, 25),
                                        new Point(10, 11)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(117),
                                Math.toRadians(45))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the FourthSampleIntake.
     *
     * @param state The DriveToGeneralSampleIntakeState to configure
     */
    public static void configFourthSampleIntake(DriveToGeneralSubmersibleIntakeState state) {
//        state.setTranslationalPIDF(0.15, 0, 0.01, 0);
//        state.setSecondaryTranslationalPIDF(0.25, 0, 0.01, 0);
//        state.setHeadingPIDF(1, 0, 0.03, 0);
//        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
//        state.setDrivePIDF(0.0025, 0, 0.00035, 0.6, 0);
//        state.setSecondaryDrivePIDF(0.008, 0, 0.0002, 0.6, 0);
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(12, 12),
                                        new Point(11, 58),
                                        new Point(49, 60)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45),
                                Math.toRadians(0))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the FourthSampleDrop.
     *
     * @param state The DriveToGeneralSampleDropState to configure
     */
    public static void configFourthSampleDrop(DriveToGeneralSampleDropState state) {
//        state.setTranslationalPIDF(0.15, 0, 0.01, 0);
//        state.setSecondaryTranslationalPIDF(0.25, 0, 0.01, 0);
//        state.setHeadingPIDF(1, 0, 0.03, 0);
//        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
//        state.setDrivePIDF(0.0025, 0, 0.00035, 0.6, 0);
//        state.setSecondaryDrivePIDF(0.008, 0, 0.0002, 0.6, 0);
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(49, 60),
                                        new Point(11,58),
                                        new Point(12, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0),
                                Math.toRadians(45))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the FifthSampleIntake.
     *
     * @param state The DriveToGeneralSampleIntakeState to configure
     */
    public static void configFifthSampleIntake(DriveToGeneralSubmersibleIntakeState state) {
        state.setTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setSecondaryTranslationalPIDF(0.25, 0, 0.01, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setDrivePIDF(0.0025, 0, 0.00035, 0.6, 0);
        state.setSecondaryDrivePIDF(0.008, 0, 0.0002, 0.6, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(12, 12),
                                        new Point(11, 58),
                                        new Point(49, 63.35)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45),
                                Math.toRadians(0))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the FifthSampleDrop.
     *
     * @param state The DriveToGeneralSampleDropState to configure
     */
    public static void configFifthSampleDrop(DriveToGeneralSampleDropState state) {
        state.setTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setSecondaryTranslationalPIDF(0.25, 0, 0.01, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setDrivePIDF(0.0025, 0, 0.00035, 0.6, 0);
        state.setSecondaryDrivePIDF(0.008, 0, 0.0002, 0.6, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(49, 63.35),
                                        new Point(11,58),
                                        new Point(12, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0),
                                Math.toRadians(45))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the DriveToSubmersible.
     *
     * @param state The DriveToSubmersible to configure
     */
    public static void configDriveToSubmersible(DriveToSubmersible state) {
        state.setTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setSecondaryTranslationalPIDF(0.25, 0, 0.01, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setDrivePIDF(0.0025, 0, 0.00035, 0.6, 0);
        state.setSecondaryDrivePIDF(0.008, 0, 0.0002, 0.6, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(10, 11),
                                        new Point(11, 58),
                                        new Point(49, 61.35)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45),
                                Math.toRadians(0))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

}
