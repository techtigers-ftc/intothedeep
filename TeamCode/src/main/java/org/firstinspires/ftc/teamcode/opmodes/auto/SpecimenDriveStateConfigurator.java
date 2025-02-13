package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenDropState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPark;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPoseState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropSpecimenState;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;

/**
 * A class used to configure Specimen Drive States.
 */
public class SpecimenDriveStateConfigurator {
    private static final double LARGE_TOLERANCE = 3;
    private static final double LARGE_ANGLE_TOLERANCE = Math.toRadians(5);
    private static final double SMALL_TOLERANCE = 1.5;
    private static final double SMALL_ANGLE_TOLERANCE = Math.toRadians(3);
    private static final double MINISCULE_TOLERANCE = 1.25;
    private static final double MICROSCOPIC_TOLERANCE = 1;
    private static final double MINISCULE_ANGLE_TOLERANCE = Math.toRadians(1.5);

    /**
     * Configures the DriveToPreloadDropState.
     *
     * @param state The DriveToPreloadDropState to configure
     */
    public static void configPreloadDrop(DriveToPreloadDropSpecimenState state) {
        state.setTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setDrivePIDF(0.02, 0, 0.0035, 0.6, 0);
        state.setHeadingPIDF(0.5, 0, 0.03, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(64.5, 7.25),
                                        new Point(66, 41.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the first intake state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configFirstIntake(DriveToFirstIntakeState state) {
        state.setTranslationalPIDF(0.1, 0, 0.001, 0);
        state.setDrivePIDF(0.008, 0, 0.001, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.03, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierCurve(
                                new Point(66, 41.5),
                                new Point(118.25, 15),
                                new Point(96, 58),
                                new Point(117.25, 58)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))

                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive state with a path chain to push the first sample into the observation zone
     *
     * @param state the state to configure
     */
    public static void configFirstPush(DriveToPoseState state) {
        state.setTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setDrivePIDF(0.02, 0, 0.0035, 0.6, 0);
        state.setHeadingPIDF(0.5, 0, 0.03, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        // Push first sample
                        .addBezierLine(
                                new Point(117.25, 58),
                                new Point(117.25, 19)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))

                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive state with a path to get to the spot for the second intake
     *
     * @param state the state to configure
     */
    public static void configSecondIntake(DriveToPoseState state) {
        state.setTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setDrivePIDF(0.02, 0, 0.0035, 0.6, 0);
        state.setHeadingPIDF(0.5, 0, 0.03, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        // Curve to second sample
                        .addBezierCurve(
                                new Point(117.25, 19),
                                new Point(100, 57),
                                new Point(124, 58)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))

                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive state with a path to push the second sample into the observation zone
     *
     * @param state the state to configure
     */
    public static void configSecondPush(DriveToPoseState state) {
        state.setTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setDrivePIDF(0.02, 0, 0.0035, 0.6, 0);
        state.setHeadingPIDF(0.5, 0, 0.03, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        // Push second sample
                        .addBezierLine(
                                new Point(124, 58),
                                new Point(124, 20)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))

                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive state with a path to go to the third intake spot
     *
     * @param state the state to configure
     */
    public static void configThirdIntake(DriveToPoseState state) {
        state.setTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setDrivePIDF(0.02, 0, 0.0035, 0.6, 0);
        state.setHeadingPIDF(0.5, 0, 0.03, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        // Curve to third sample
                        .addBezierCurve(
                                new Point(124, 20),
                                new Point(110.5, 57),
                                new Point(134, 58)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))

                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive state with a path chain to push the third sample into the observation zone
     *
     * @param state the state to configure
     */
    public static void configThirdPush(DriveToPoseState state) {
        state.setTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setDrivePIDF(0.02, 0, 0.0035, 0.6, 0);
        state.setHeadingPIDF(0.5, 0, 0.03, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        // Push third sample
                        .addBezierCurve(
                                new Point(134, 58),
                                new Point(134, 20)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))

                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive to the first specimen intake from the wall
     *
     * @param state the state to configure
     */
    public static void configFirstSpecimenIntake(DriveToPoseState state) {
        state.setTranslationalPIDF(0.125, 0, 0.01, 0);
        state.setDrivePIDF(0.005, 0, 0.0035, 0.6, 0);
        state.setHeadingPIDF(0.5, 0, 0.03, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierCurve(
                                new Point(134, 20),
                                new Point(125, 15),
                                new Point(129, 6.5)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(MINISCULE_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the first specimen drop state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configFirstSpecimenDrop(DriveToGeneralSpecimenDropState state) {
//        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
//        state.setDrivePIDF(0.005, 0, 0.0007, 0.6, 0);
//        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(129, 7),
                                new Point(68, 42.5)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the second specimen intake state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configSecondSpecimenIntake(DriveToGeneralSpecimenIntakeState state) {
//        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
//        state.setDrivePIDF(0.005, 0, 0.0007, 0.6, 0);
//        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierCurve(
                                new Point(68, 42.5),
                                new Point(73.25, 16),
                                new Point(116.25, 40),
                                new Point(111, 6.5)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the second specimen drop state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configSecondSpecimenDrop(DriveToGeneralSpecimenDropState state) {
//        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
//        state.setDrivePIDF(0.005, 0, 0.0007, 0.6, 0);
//        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(111, 7),
                                new Point(70, 42.5)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the third specimen intake state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configThirdSpecimenIntake(DriveToGeneralSpecimenIntakeState state) {
//        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
//        state.setDrivePIDF(0.005, 0, 0.0007, 0.6, 0);
//        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierCurve(
                                new Point(70, 42.5),
                                new Point(73.25, 16),
                                new Point(116.25, 40),
                                new Point(111, 6.5)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the third specimen drop state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configThirdSpecimenDrop(DriveToGeneralSpecimenDropState state) {
//        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
//        state.setDrivePIDF(0.005, 0, 0.0007, 0.6, 0);
//        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(111, 7),
                                new Point(72, 42.5)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the fourth specimen intake state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configFourthSpecimenIntake(DriveToGeneralSpecimenIntakeState state) {
//        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
//        state.setDrivePIDF(0.005, 0, 0.0007, 0.6, 0);
//        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierCurve(
                                new Point(72, 42.5),
                                new Point(73.25, 16),
                                new Point(116.25, 40),
                                new Point(111, 6.5)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the fourth specimen drop state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configFourthSpecimenDrop(DriveToGeneralSpecimenDropState state) {
//        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
//        state.setDrivePIDF(0.005, 0, 0.0007, 0.6, 0);
//        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(111, 7),
                                new Point(74, 42.5)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(SMALL_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
    }

    /**
     * Configures the drive to park state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configDriveToPark(DriveToPark state) {
//        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
//        state.setDrivePIDF(0.005, 0, 0.0007, 0.6, 0);
//        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(74, 42.5),
                                new Point(93.5, 26)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90),
                                Math.toRadians(-45))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }
}
