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
        state.setTranslationalPIDF(TuningConstants.translationalP, TuningConstants.translationalI, TuningConstants.translationalD, 0);
        //state.setSecondaryTranslationalPIDF(TuningConstants.translationalD, TuningConstants.S, TuningConstants.SECONDARY_TRANSLATIONAL_D, 0);
        state.setHeadingPIDF(TuningConstants.headingP, TuningConstants.headingI, TuningConstants.headingD, 0);
        //state.setSecondaryHeadingPIDF(TuningConstants.SECONDARY_HEADING_P, TuningConstants.SECONDARY_HEADING_I, TuningConstants.SECONDARY_HEADING_D, 0);
        state.setDrivePIDF(TuningConstants.driveP, TuningConstants.driveI, TuningConstants.driveD, TuningConstants.driveT, TuningConstants.driveF);
        //state.setSecondaryDrivePIDF(TuningConstants.SECONDARY_DRIVE_P, TuningConstants.SECONDARY_DRIVE_I, TuningConstants.SECONDARY_DRIVE_D, TuningConstants.SECONDARY_DRIVE_F, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(9.75, 77),
                                        new Point(44, 77)
                                )
                        )
                        .setConstantHeadingInterpolation(90)
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    public static void configDriveToFirstPush(DriveToFirstPush state) {
        state.setTranslationalPIDF(TuningConstants.translationalP, TuningConstants.translationalI, TuningConstants.translationalD, 0);
        //state.setSecondaryTranslationalPIDF(TuningConstants.translationalD, TuningConstants.S, TuningConstants.SECONDARY_TRANSLATIONAL_D, 0);
        state.setHeadingPIDF(TuningConstants.headingP, TuningConstants.headingI, TuningConstants.headingD, 0);
        //state.setSecondaryHeadingPIDF(TuningConstants.SECONDARY_HEADING_P, TuningConstants.SECONDARY_HEADING_I, TuningConstants.SECONDARY_HEADING_D, 0);
        state.setDrivePIDF(TuningConstants.driveP, TuningConstants.driveI, TuningConstants.driveD, TuningConstants.driveT, TuningConstants.driveF);
        //state.setSecondaryDrivePIDF(TuningConstants.SECONDARY_DRIVE_P, TuningConstants.SECONDARY_DRIVE_I, TuningConstants.SECONDARY_DRIVE_D, TuningConstants.SECONDARY_DRIVE_F, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(44, 77),
                                        new Point(35, 104),
                                        new Point(60, 120)
                                )
                        )
                        .setConstantHeadingInterpolation(90)
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    public static void configFirstPush(FirstPush state) {
        state.setTranslationalPIDF(TuningConstants.translationalP, TuningConstants.translationalI, TuningConstants.translationalD, 0);
        //state.setSecondaryTranslationalPIDF(TuningConstants.translationalD, TuningConstants.S, TuningConstants.SECONDARY_TRANSLATIONAL_D, 0);
        state.setHeadingPIDF(TuningConstants.headingP, TuningConstants.headingI, TuningConstants.headingD, 0);
        //state.setSecondaryHeadingPIDF(TuningConstants.SECONDARY_HEADING_P, TuningConstants.SECONDARY_HEADING_I, TuningConstants.SECONDARY_HEADING_D, 0);
        state.setDrivePIDF(TuningConstants.driveP, TuningConstants.driveI, TuningConstants.driveD, TuningConstants.driveT, TuningConstants.driveF);
        //state.setSecondaryDrivePIDF(TuningConstants.SECONDARY_DRIVE_P, TuningConstants.SECONDARY_DRIVE_I, TuningConstants.SECONDARY_DRIVE_D, TuningConstants.SECONDARY_DRIVE_F, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(60, 120),
                                        new Point(12, 120)
                                )
                        )
                        .setConstantHeadingInterpolation(90)
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }
}
