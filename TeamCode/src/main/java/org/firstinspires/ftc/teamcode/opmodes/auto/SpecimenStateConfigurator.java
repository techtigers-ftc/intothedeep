package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.basket.DriveToPreloadDropState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropStateSpecimen;
import org.firstinspires.ftc.teamcode.autostates.specimen.GeneralDriveToPush;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;

public class SpecimenStateConfigurator {

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
        state.setTranslationalPIDF(0.14, 0, 0.02, 0);
        state.setHeadingPIDF(0.6, 0, 0, 0);
        state.setDrivePIDF(0.0026, 0, 0.001, 0, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(77, 7.25),
                                        new Point(77, 41.25)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(5);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the DriveToPreloadDropState.
     *
     * @param state The DriveToPreloadDropState to configure
     */
    public static void configFirstPushDrive(GeneralDriveToPush state) {
        state.setTranslationalPIDF(TuningConstants.translationalP,
                TuningConstants.translationalI,
                TuningConstants.translationalD, 0);
        state.setHeadingPIDF(TuningConstants.headingP, TuningConstants.headingI,
                TuningConstants.headingD, 0);
        state.setDrivePIDF(TuningConstants.driveP, TuningConstants.driveI,
                TuningConstants.driveD, 0, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(77, 48.25),
                                        new Point(109, 40),
                                        new Point (84, 24)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }
}
