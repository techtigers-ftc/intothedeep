package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstIntake;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPlace;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropStateSpecimen;
import org.firstinspires.ftc.teamcode.autostates.specimen.SecondDropOff;
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
    private static final double MINISCULE_TOLERANCE = 0.85;
    private static final double MINISCULE_ANGLE_TOLERANCE = Math.toRadians(2);

    /**
     * Configures the DriveToPreloadDropState.
     *
     * @param state The DriveToPreloadDropState to configure
     */
    public static void configPreloadDrop(DriveToPreloadDropStateSpecimen state) {
        state.setTranslationalPIDF(0.1, 0, 0.01, 0);
        state.setDrivePIDF(0.006, 0, 0.0004, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.15, 0, 0.01, 0);


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
        state.setPIDSToTuning();
        state.setDrivePIDF(0.0025,0,0.0003,0.6,0);
        state.setTranslationalPIDF(0.3,0,0.01,0);
        state.setHeadingPIDF(1.5, 0,0.3,0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.2, 0, 0.01, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(74, 40.25),
                                        new Point(130, 23.5)
                                )
                        )

                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                        .build()
        );

        state.setTolerance(MINISCULE_TOLERANCE);
        state.setAngleTolerance(MINISCULE_ANGLE_TOLERANCE);
    }

    public static void configFirstDrop(DriveToPlace state) {
        state.setDrivePIDF(0.003,0,0.00035,0.6,0);
        state.setTranslationalPIDF(0.2,0,0.01,0);
        state.setHeadingPIDF(0.35, 0,0.02,0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(0.35, 0, 0.02, 0);
        state.setSecondaryTranslationalPIDF(0.15, 0, 0.01, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(106.5, 27.5),
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
        state.setPIDSToTuning();
        state.setDrivePIDF(0.004,0,0.00035,0.6,0);
        state.setTranslationalPIDF(0.2,0,0.01,0);
        state.setHeadingPIDF(0.5, 0,0.1,0);
        state.setSecondaryDrivePIDF(0.005, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(0.7, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.2, 0, 0.01, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(105, 27),
                                        new Point(117, 27.5)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-40),
                                Math.toRadians(55))
                        .build()
        );

        state.setTolerance(MINISCULE_TOLERANCE);
        state.setAngleTolerance(MINISCULE_ANGLE_TOLERANCE);
    }

    public static void configSecondDrop(SecondDropOff state) {
        state.setPIDSToTuning();
        state.setDrivePIDF(0.003,0,0.00035,0.6,0);
        state.setTranslationalPIDF(0.2,0,0.01,0);
        state.setHeadingPIDF(0.35, 0,0.02,0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(0.35, 0, 0.02, 0);
        state.setSecondaryTranslationalPIDF(0.15, 0, 0.01, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(117, 27.5),
                                        new Point(114, 27)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(55),
                                Math.toRadians(-40))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    public static void configThirdIntake(SecondDropOff state) {
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(114, 27),
//                                        new Point(110, 24),
                                        new Point(121, 32)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-40),
                                Math.toRadians(39))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    public static void configThirdDrop(DriveToPlace state) {
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(119, 29),
//                                        new Point(110, 24),
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
