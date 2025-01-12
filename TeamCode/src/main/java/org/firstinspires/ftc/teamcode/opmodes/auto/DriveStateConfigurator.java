package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.DriveToGeneralDropState;
import org.firstinspires.ftc.teamcode.autostates.DriveToIntakeState;
import org.firstinspires.ftc.teamcode.autostates.DriveToPreloadDropState;
import org.firstinspires.ftc.teamcode.autostates.DriveToSubmersible;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;

/**
 * A class used to configure DriveStates.
 */
public class DriveStateConfigurator {
    private static final double LARGE_TOLERANCE = 3;
    private static final double LARGE_ANGLE_TOLERANCE = Math.toRadians(5);
    private static final double SMALL_TOLERANCE = 1.5;
    private static final double SMALL_ANGLE_TOLERANCE = Math.toRadians(3);

    /**
     * Configures the DriveToPreloadDropState.
     *
     * @param state The DriveToPreloadDropState to configure
     */
    public static void configPreloadDrop(DriveToPreloadDropState state) {
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
                                        new Point(29.75, 7.25),
                                        new Point(12, 12)
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
    public static void configFirstSampleIntake(DriveToIntakeState state) {
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
                                        new Point(12, 12),
                                        new Point(14, 23)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45),
                                Math.toRadians(71))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the FirstSampleDrop.
     *
     * @param state The FirstSampleDrop to configure
     */
    public static void configFirstSampleDrop(DriveToGeneralDropState state) {
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
                                        new Point(16, 20),
                                        new Point(12, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(78),
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
    public static void configSecondSampleIntake(DriveToIntakeState state) {
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
                                        new Point(12, 12),
                                        new Point(12, 21)
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
    public static void configSecondSampleDrop(DriveToGeneralDropState state) {
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
                                        new Point(12, 21),
                                        new Point(12, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90),
                                Math.toRadians(45))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the ThirdSampleIntake.
     *
     * @param state The ThirdSampleIntake to configure
     */
    public static void configThirdSampleIntake(DriveToIntakeState state) {
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
                                        new Point(12, 12),
                                        new Point(14, 22.75)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45),
                                Math.toRadians(122))
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
    public static void configThirdSampleDrop(DriveToGeneralDropState state) {
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
                                        new Point(14, 22.75),
                                        new Point(12, 12)
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
     * Configures the DriveToSubmersible.
     *
     * @param state The DriveToSubmersible to configure
     */
    public static void configDriveToSubmersible(DriveToSubmersible state) {
        state.setTranslationalPIDF(0.25, 0, 0.01, 0);
        state.setSecondaryTranslationalPIDF(0.25, 0, 0.01, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setDrivePIDF(0.005, 0, 0.00035, 0.6, 0);
        state.setSecondaryDrivePIDF(0.005, 0, 0.0002, 0.6, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(12, 12),
                                        new Point(11, 58),
                                        new Point(51, 61.35)
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
