package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstIntake;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenDropState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPark;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DropSampleState;
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
    private static final double MINISCULE_ANGLE_TOLERANCE = Math.toRadians(1.5);

    /**
     * Configures the DriveToPreloadDropState.
     *
     * @param state The DriveToPreloadDropState to configure
     */
    public static void configPreloadDrop(DriveToPreloadDropSpecimenState state) {
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
        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
        state.setDrivePIDF(0.003, 0, 0.00055, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.175, 0, 0.03, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(74, 40.25),
                                        new Point(130, 26.5)
                                )
                        )

                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                        .build()
        );

        state.setTolerance(MINISCULE_TOLERANCE);
        state.setAngleTolerance(MINISCULE_ANGLE_TOLERANCE);
    }

    public static void configureFirstHoldPoint(DropSampleState state) {
        state.setTolerance(MINISCULE_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
        state.setDrivePIDF(0.003, 0, 0.00055, 0.6, 0);
        state.setHeadingPIDF(2, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.175, 0, 0.03, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(2.5, 0, 0.03, 0);
        state.setTargetPosition(129, 23.5, Math.toRadians(90));
    }

    public static void configureSecondHoldPoint(DropSampleState state) {
        state.setTolerance(MINISCULE_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
        state.setDrivePIDF(0.003, 0, 0.00055, 0.6, 0);
        state.setHeadingPIDF(2, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.175, 0, 0.03, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(2.5, 0, 0.03, 0);
        state.setTargetPosition(129, 23.5, Math.toRadians(65));
    }

    public static void configFirstSpecimenIntake(DriveToGeneralSpecimenIntakeState state) {
        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
        state.setDrivePIDF(0.003, 0, 0.00055, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.175, 0, 0.03, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(129, 23.5),
                                new Point(95, 23)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90),
                                Math.toRadians(-45))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    public static void configFirstSpecimenDrop(DriveToGeneralSpecimenDropState state) {
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(95, 23),
                                new Point(71, 41)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45),
                                Math.toRadians(-90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    public static void configSecondSpecimenIntake(DriveToGeneralSpecimenIntakeState state) {
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(71, 41),
                                new Point(95, 23)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-90),
                                Math.toRadians(-45))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    public static void configSecondSpecimenDrop(DriveToGeneralSpecimenDropState state) {
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(95, 23),
                                new Point(69, 41)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45),
                                Math.toRadians(-90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    public static void configThirdSpecimenIntake(DriveToGeneralSpecimenIntakeState state) {
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(69, 41),
                                new Point(95, 23)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-90),
                                Math.toRadians(-45))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    public static void configThirdSpecimenDrop(DriveToGeneralSpecimenDropState state) {
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(95, 23),
                                new Point(67, 41)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45),
                                Math.toRadians(-90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    public static void configFourthSpecimenIntake(DriveToGeneralSpecimenIntakeState state) {
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(67, 41),
                                new Point(95, 23)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-90),
                                Math.toRadians(-45))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    public static void configFourthSpecimenDrop(DriveToGeneralSpecimenDropState state) {
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(95, 23),
                                new Point(65, 41)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45),
                                Math.toRadians(-90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    public static void configDriveToPark(DriveToPark state) {
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(65, 41),
                                new Point(95, 23)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-90),
                                Math.toRadians(-45))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }
}
