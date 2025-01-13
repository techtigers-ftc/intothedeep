package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstPush;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropStateSpecimen;
import org.firstinspires.ftc.teamcode.autostates.specimen.FirstPush;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;

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

    public static void configDriveToFirstPush(DriveToFirstPush state) {
        state.setPIDSToDefaultValues();
        state.setDrivePIDF(0.008, 0, 0.00035, 0.6, 0);
        state.setTranslationalPIDF(0.35, 0, 0.01, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(74, 40.25),
                                        new Point(95, 19),
                                        new Point(105, 19),
                                        new Point(106, 72),
                                        new Point(115, 60)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    public static void configFirstPush(FirstPush state) {
        state.setPIDSToDefaultValues();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(120, 60),
                                        new Point(120, 12)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }
}
