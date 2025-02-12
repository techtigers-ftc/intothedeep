package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenDropState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPark;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPoseState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DropSampleState;
import org.firstinspires.ftc.teamcode.autostates.specimen.IntakeSpecimenHoldPointState;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
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
    private static final double MINISCULE_TOLERANCE = 0.85;
    private static final double MINISCULE_ANGLE_TOLERANCE = Math.toRadians(1.5);

    /**
     * Configures the DriveToPreloadDropState.
     *
     * @param state The DriveToPreloadDropState to configure
     */
    public static void configPreloadDrop(DriveToPreloadDropSpecimenState state) {
        state.setTranslationalPIDF(0.15, 0, 0.01, 0);
        state.setDrivePIDF(0.014, 0, 0.0035, 0.6, 0);
        state.setHeadingPIDF(0.5, 0, 0.03, 0);
//        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(77, 7.25),
                                        new Point(76, 41.5)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the first intake state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configFirstIntake(DriveToPoseState state) {
//        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
//        state.setDrivePIDF(0.003, 0, 0.00055, 0.6, 0);
//        state.setHeadingPIDF(1, 0, 0.06, 0);
//        state.setSecondaryTranslationalPIDF(0.175, 0, 0.03, 0);
//        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
//        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);
//        state.setPIDSToTuning();
        state.setPrimaryPIDSToTuning();

        state.setPathChain(
                new PathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(76, 41.5),
                                        new Point(118.25, 15),
                                        new Point(96, 58),
                                        new Point(116.25, 58)
                                )
                        )

                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the first hold point with PIDF coefficients and a target position
     *
     * @param state the state to configure
     */
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

    /**
     * Configures the second hold point with PIDF coefficients and a target position
     *
     * @param state the state to configure
     */
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

    /**
     * Configures the third hold point with PIDF coefficients and a target position
     *
     * @param state the state to configure
     */
    public static void configureThirdHoldPoint(DropSampleState state) {
        configureFirstHoldPoint(state);
    }

    /**
     * Configures the first specimen intake state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
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
                                new Point(93.5, 26)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90),
                                Math.toRadians(-45))
                        .build()
        );

        state.setTolerance(MINISCULE_TOLERANCE);
        state.setAngleTolerance(MINISCULE_ANGLE_TOLERANCE);
    }

    /**
     * Configures the specimen intake (hold point) state with PIDF coefficients and a target position
     *
     * @param state the state to configure
     */
    public static void configSpecimenIntakeHoldPoint(IntakeSpecimenHoldPointState state) {
        state.setTolerance(MINISCULE_TOLERANCE);
        state.setAngleTolerance(SMALL_ANGLE_TOLERANCE);
        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
        state.setDrivePIDF(0.003, 0, 0.00055, 0.6, 0);
        state.setHeadingPIDF(2, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.175, 0, 0.03, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(2.5, 0, 0.03, 0);
        state.setTargetPosition(93.5, 26, Math.toRadians(-45));
    }

    /**
     * Configures the first specimen drop state with PIDF coefficients and a path
     *
     * @param state the state to configure
     */
    public static void configFirstSpecimenDrop(DriveToGeneralSpecimenDropState state) {
        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
        state.setDrivePIDF(0.005, 0, 0.0007, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.25, 0, 0.025, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(93.5, 26),
                                new Point(74, 42.5)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45),
                                Math.toRadians(-90))
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
        state.setTranslationalPIDF(0.3, 0, 0.02, 0);
        state.setDrivePIDF(0.004, 0, 0.00065, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.1, 0, 0.03, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0006, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(74, 42.5),
                                new Point(93.5, 26)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-90),
                                Math.toRadians(-45))
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
        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
        state.setDrivePIDF(0.005, 0, 0.0007, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.25, 0, 0.025, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(93.5, 26),
                                new Point(71, 42.5)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45),
                                Math.toRadians(-90))
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
        state.setTranslationalPIDF(0.3, 0, 0.02, 0);
        state.setDrivePIDF(0.004, 0, 0.00065, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.1, 0, 0.03, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0006, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(71, 42.5),
                                new Point(93.5, 26)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-90),
                                Math.toRadians(-45))
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
        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
        state.setDrivePIDF(0.005, 0, 0.0007, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.25, 0, 0.025, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(93.5, 26),
                                new Point(67, 42.5)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45),
                                Math.toRadians(-90))
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
        state.setTranslationalPIDF(0.3, 0, 0.02, 0);
        state.setDrivePIDF(0.004, 0, 0.00065, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.1, 0, 0.03, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0006, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(67, 42.5),
                                new Point(93.5, 26)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-90),
                                Math.toRadians(-45))
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
        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
        state.setDrivePIDF(0.005, 0, 0.0007, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.25, 0, 0.025, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);

        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(93.5, 26),
                                new Point(66, 42.5)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45),
                                Math.toRadians(-90))
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
        state.setTranslationalPIDF(0.3, 0, 0.01, 0);
        state.setDrivePIDF(0.005, 0, 0.0006, 0.6, 0);
        state.setHeadingPIDF(1, 0, 0.06, 0);
        state.setSecondaryTranslationalPIDF(0.25, 0, 0.025, 0);
        state.setSecondaryDrivePIDF(0.004, 0, 0.0002, 0.6, 0);
        state.setSecondaryHeadingPIDF(1, 0, 0.06, 0);


        state.setPathChain(
                new PathBuilder()
                        .addBezierLine(
                                new Point(67, 42.5),
                                new Point(93.5, 26)
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-90),
                                Math.toRadians(-45))
                        .build()
        );

        state.setTolerance(LARGE_TOLERANCE);
        state.setAngleTolerance(LARGE_ANGLE_TOLERANCE);
    }
}
