package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstIntake;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPlace;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropStateSpecimen;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;

/**
 * A class used to configure DriveStates.
 */
public class SpecimenDriveStateConfigurator {
    private static final double LARGE_TOLERANCE = 3;
    private static final double LARGE_ANGLE_TOLERANCE = Math.toRadians(5);
    private static final double SMALL_TOLERANCE = 1.5;
    private static final double SMALL_ANGLE_TOLERANCE = Math.toRadians(3);

    /**
     * Configures the DriveToPreloadDropState.
     *
     * @param state The DriveToPreloadDropState to configure
     */
    public static void configPreloadDrop(DriveToPreloadDropStateSpecimen state) {
        state.setPIDSToDefaultValues();
        state.setDrivePIDF(0.006, 0, 0.0004, 0.6, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(77, 7.25),
                                        new Point(74, 40.25)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    public static void configFirstIntake(DriveToFirstIntake state) {
        state.setPIDSToDefaultValues();
        state.setDrivePIDF(0.004,0,0.00035,0.6,0);
        state.setTranslationalPIDF(0.2,0,0.01,0);
        state.setHeadingPIDF(1.1, 0,0.3,0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(74, 40.25),
                                        new Point(102.5, 27)
                                )
                        )

                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(55))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    public static void configFirstDrop(DriveToPlace state) {
        state.setPIDSToDefaultValues();
        state.setHeadingPIDF(0.5, 0,0.03,0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(102.5, 27),
                                        new Point(105, 27)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(-40))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    public static void configSecondIntake(DriveToPlace state) {
        state.setPIDSToDefaultValues();
        state.setDrivePIDF(0.004,0,0.00035,0.6,0);
        state.setTranslationalPIDF(0.2,0,0.01,0);
        state.setHeadingPIDF(0.5, 0,0.3,0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(105, 27),
                                        new Point(112.5, 27)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-40),
                                Math.toRadians(55))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    public static void configSecondDrop(DriveToPlace state) {
        state.setPIDSToDefaultValues();
        state.setHeadingPIDF(0.5, 0,0.03,0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(112.5, 27),
                                        new Point(115, 27)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(55),
                                Math.toRadians(-40))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    public static void configThirdIntake(DriveToPlace state) {
        state.setPIDSToDefaultValues();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(115, 27),
                                        new Point(120, 27)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-40),
                                Math.toRadians(35))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    public static void configThirdDrop(DriveToPlace state) {
        state.setPIDSToDefaultValues();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(120, 27),
                                        new Point(110, 27)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(35),
                                Math.toRadians(-40))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }
}
