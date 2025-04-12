package org.firstinspires.ftc.teamcode.opmodes.auto.configurators;

import org.firstinspires.ftc.teamcode.autostates.basket.DriveFromSubmersibleSampleDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSampleDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSampleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSubmersibleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToSamplePark;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToPreloadDropState;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;

/**
 * A class used to configure BasketDriveStates.
 */
public class BasketDriveStateConfigurator {
    public static final double MEGA_TOLERANCE = 7;
    public static final double MEGA_ANGLE_TOLERANCE = Math.toRadians(10);
    public static final double LARGE_TOLERANCE = 3;
    public static final double LARGE_ANGLE_TOLERANCE = Math.toRadians(5);
    public static final double SMALL_TOLERANCE = 2;
    public static final double SMALL_ANGLE_TOLERANCE = Math.toRadians(3);

    /**
     * Configures the DriveToPreloadDropState
     *
     * @param state The DriveToPreloadDropState to configure
     */
    public static void configPreloadDrop(DriveToPreloadDropState state) {
        state.setTranslationalPIDF(0.08, 0, 0.001, 0);
        state.setDrivePIDF(0.004, 0, 0.002, 0.6, 0);
        state.setHeadingPIDF(0.9, 0, 0.015, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(29.75, 7.25),
                                        new Point(7, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90),
                                Math.toRadians(72))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the DriveToFirstSampleIntakeState.
     *
     * @param state The DriveToFirstSampleIntakeState to configure
     */
    public static void configFirstSampleIntake(DriveToGeneralSampleIntakeState state) {
        state.setTranslationalPIDF(0.08, 0, 0.001, 0);
        state.setHeadingPIDF(0.5, 0, 0.035, 0);
        state.setDrivePIDF(0.0025, 0, 0.0035, 0.6, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(7, 12),
                                        new Point(14.5, 19)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72),
                                Math.toRadians(72))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the FirstSampleDrop.
     *
     * @param state The FirstSampleDrop to configure
     */
    public static void configFirstSampleDrop(DriveToGeneralSampleDropState state) {
        state.setTranslationalPIDF(0.1, 0, 0.01, 0);
        state.setDrivePIDF(0.003, 0, 0.00035, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(14.5, 19),
                                        new Point(9, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72),
                                Math.toRadians(72))
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
        state.setTranslationalPIDF(0.12, 0, 0.001, 0);
        state.setHeadingPIDF(0.7, 0, 0.035, 0);
        state.setDrivePIDF(0.001, 0, 0.0035, 0.6, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(9, 12),
                                        new Point(11.5, 17)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72),
                                Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the SecondSampleDrop.
     *
     * @param state The SecondSampleDrop to configure
     */
    public static void configSecondSampleDrop(DriveToGeneralSampleDropState state) {
        state.setTranslationalPIDF(0.1, 0, 0.01, 0);
        state.setDrivePIDF(0.003, 0, 0.00035, 0.6, 0);
        state.setHeadingPIDF(2, 0, 0, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(11.5, 17),
                                        new Point(8, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90),
                                Math.toRadians(75))
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
        state.setTranslationalPIDF(0.12, 0, 0.001, 0);
        state.setHeadingPIDF(0.7, 0, 0.035, 0);
        state.setDrivePIDF(0.001, 0, 0.0035, 0.6, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(8, 12),
                                        new Point(10.5, 19)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(75),
                                Math.toRadians(108))
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
        state.setDrivePIDF(0.003, 0, 0.00035, 0.6, 0);
        state.setHeadingPIDF(2, 0, 0, 0);
//        state.setPrimaryPIDSToTuning();


        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(10.5, 19),
                                        new Point(9, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(108),
                                Math.toRadians(72))
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
        state.setTranslationalPIDF(0.15, 0, 0, 0);
        state.setDrivePIDF(0.01, 0, 0.007, 0.6, 0);
        state.setHeadingPIDF(1.25, 0, 0, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(9, 12),
                                        new Point(25, 60),
                                        new Point(47, 62.5)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72),
                                Math.toRadians(0))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the FourthSampleDrop.
     *
     * @param state The DriveToGeneralSampleDropState to configure
     */
    public static void configFourthSampleDrop(DriveFromSubmersibleSampleDropState state) {
        state.setTranslationalPIDF(0.15, 0, 0, 0);
        state.setHeadingPIDF(1.25, 0, 0, 0);
        state.setDrivePIDF(0.01, 0, 0.00475, 0.6, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(47, 62.5),
                                        new Point(35, 60),
                                        new Point(8, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0),
                                Math.toRadians(72))
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
        state.setTranslationalPIDF(0.15, 0, 0, 0);
        state.setHeadingPIDF(1.25, 0, 0, 0);
        state.setDrivePIDF(0.01, 0, 0.007, 0.6, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(8, 12),
                                        new Point(25, 60),
                                        new Point(47, 64.5)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72),
                                Math.toRadians(0))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the FifthSampleDrop.
     *
     * @param state The DriveToGeneralSampleDropState to configure
     */
    public static void configFifthSampleDrop(DriveFromSubmersibleSampleDropState state) {
        state.setTranslationalPIDF(0.15, 0, 0, 0);
        state.setHeadingPIDF(1.25, 0, 0, 0);
        state.setDrivePIDF(0.01, 0, 0.00475, 0.6, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(47, 64.5),
                                        new Point(35, 60),
                                        new Point(8, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0),
                                Math.toRadians(72))
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
    public static void configSixthSampleIntake(DriveToGeneralSubmersibleIntakeState state) {
        state.setTranslationalPIDF(0.15, 0, 0, 0);
        state.setHeadingPIDF(1.25, 0, 0, 0);
        state.setDrivePIDF(0.01, 0, 0.007, 0.6, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(8, 12),
                                        new Point(25, 60),
                                        new Point(47, 66.5)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72),
                                Math.toRadians(0))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the FifthSampleDrop.
     *
     * @param state The DriveToGeneralSampleDropState to configure
     */
    public static void configSixthSampleDrop(DriveFromSubmersibleSampleDropState state) {
        state.setTranslationalPIDF(0.15, 0, 0, 0);
        state.setHeadingPIDF(1.25, 0, 0, 0);
        state.setDrivePIDF(0.01, 0, 0.00475, 0.6, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(47, 66.5),
                                        new Point(35, 60),
                                        new Point(8, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0),
                                Math.toRadians(72))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }



    /**
     * Configures the DriveToPark.
     *
     * @param state The DriveToPark to configure
     */
    public static void configDriveToPark(DriveToSamplePark state) {
        state.setTranslationalPIDF(0.15, 0, 0, 0);
        state.setDrivePIDF(0.01, 0, 0.005, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(8, 12),
                                        new Point(25, 60),
                                        new Point(49, 61)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72),
                                Math.toRadians(0))
                        .build()
        );

        state.setTolerance(MEGA_TOLERANCE);
        state.setAngleTolerance(MEGA_ANGLE_TOLERANCE);
    }
}
