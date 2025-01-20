package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstIntake;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenDropState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPark;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPlace;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DropSampleState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DropSecondAndThirdSampleState;
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
//        state.setPIDSToTuning();
        state.setDrivePIDF(0.003, 0, 0.00035, 0.6, 0);
        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.175, 0, 0.025, 0);

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

    public static void configureFirstHoldPoint(DropSampleState state) {
        state.setTolerance(0.5);
        state.setAngleTolerance(Math.toRadians(1));
        state.setPIDSToTuning();
        state.setTargetPosition(128, 23.5, Math.toRadians(90));
    }

    public static void configureSecondHoldPoint(DropSecondAndThirdSampleState state) {
        state.setTolerance(0.5);
        state.setAngleTolerance(Math.toRadians(1));
        state.setPIDSToTuning();
        state.setTargetPosition(130, 23.5, Math.toRadians(90));
    }

    public static void configureThirdHoldPoint(DropSecondAndThirdSampleState state) {
        state.setTolerance(0.5);
        state.setAngleTolerance(Math.toRadians(1));
        state.setPIDSToTuning();
        state.setTargetPosition(130, 23.5, Math.toRadians(90));
    }

    public static void configFirstSpecimenIntake(DriveToPlace state) {
        state.setPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(121, 32),
                                new Point(95, 23)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(39),
                                Math.toRadians(315))
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
                                new Point(71, 40.25)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(315),
                                Math.toRadians(270))
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
                                new Point(71, 40.25),
                                new Point(100.5, 18)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(270),
                                Math.toRadians(315))
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
                                new Point(100.5, 18),
                                new Point(68, 40.25)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(315),
                                Math.toRadians(270))
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
                                new Point(68, 40.25),
                                new Point(100.5, 18)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(270),
                                Math.toRadians(315))
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
                                new Point(100.5, 18),
                                new Point(65, 40.25)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(315),
                                Math.toRadians(270))
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
                                new Point(65, 40.25),
                                new Point(100.5, 18)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(270),
                                Math.toRadians(315))
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
                                new Point(100.5, 18),
                                new Point(62, 40.25)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(315),
                                Math.toRadians(270))
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
                                new Point(62, 40.25),
                                new Point(95, 23)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(270),
                                Math.toRadians(315))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }
}
