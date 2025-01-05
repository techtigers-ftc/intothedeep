package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.DriveToGeneralDropState;
import org.firstinspires.ftc.teamcode.autostates.DriveToPreloadDropState;
import org.firstinspires.ftc.teamcode.autostates.DriveToIntakeState;
import org.firstinspires.ftc.teamcode.autostates.DriveToSubmersible;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;

/**
 * A class used to configure DriveStates.
 */
public class DriveStateConfigurator {
    /**
     * Configures the DriveToPreloadDropState.
     *
     * @param state The DriveToPreloadDropState to configure
     */
    public static void configPreloadDrop(DriveToPreloadDropState state) {
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
                                new BezierLine(
                                        new Point(29.75, 7.25),
                                        new Point(12, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90),
                                Math.toRadians(45))
                        .build()
        );
    }

    /**
     * Configures the DriveToFirstSampleIntakeState.
     *
     * @param state The DriveToFirstSampleIntakeState to configure
     */
    public static void configFirstSampleIntake(DriveToIntakeState state) {
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
                                new BezierLine(
                                        new Point(12, 12),
                                        new Point(14, 23)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45),
                                Math.toRadians(71))
                        .build()
        );
    }

    /**
     * Configures the FirstSampleDrop.
     *
     * @param state The FirstSampleDrop to configure
     */
    public static void configFirstSampleDrop(DriveToGeneralDropState state) {
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
                                new BezierLine(
                                        new Point(16, 20),
                                        new Point(12, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(78),
                                Math.toRadians(45))
                        .build()
        );
    }

    /**
     * Configures the SecondSampleIntake.
     *
     * @param state The SecondSampleIntake to configure
     */
    public static void configSecondSampleIntake(DriveToIntakeState state) {
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
                                new BezierLine(
                                        new Point(12, 12),
                                        new Point(12, 21)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45),
                                Math.toRadians(90))
                        .build()
        );
    }

    /**
     * Configures the ThirdSampleDrop.
     *
     * @param state The ThirdSampleDrop to configure
     */
    public static void configThirdSampleDrop(DriveToGeneralDropState state) {
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
                                new BezierLine(
                                        new Point(13.5, 22.75),
                                        new Point(12, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(117),
                                Math.toRadians(45))
                        .build()
        );
    }

    /**
     * Configures the SecondSampleDrop.
     *
     * @param state The SecondSampleDrop to configure
     */
    public static void configSecondSampleDrop(DriveToGeneralDropState state) {
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
                                new BezierLine(
                                        new Point(12, 21),
                                        new Point(12, 12)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90),
                                Math.toRadians(45))
                        .build()
        );
    }

    /**
     * Configures the ThirdSampleIntake.
     *
     * @param state The ThirdSampleIntake to configure
     */
    public static void configThirdSampleIntake(DriveToIntakeState state) {
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
                                new BezierLine(
                                        new Point(12, 12),
                                        new Point(13.5, 22.75)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45),
                                Math.toRadians(117))
                        .build()
        );
    }

    /**
     * Configures the DriveToSubmersible.
     * @param state The DriveToSubmersible to configure
     */
    public static void configDriveToSubmersible(DriveToSubmersible state) {
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
                                        new Point(12, 12),
                                        new Point( 14.82, 54.67),
                                        new Point( 48.42, 61.35)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45),
                                Math.toRadians(0))
                        .build()
        );
    }
}
