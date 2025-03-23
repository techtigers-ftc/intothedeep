package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.commands.TimeoutCommand;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.localizers.RobotStateLocalizer;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.FilteredPIDFController;
import org.firstinspires.ftc.teamcode.pedropathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseTranslator;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import java.util.function.DoubleSupplier;

import team.techtigers.core.paths.Waypoint;

/**
 * A class for autonomous drive commands that use PedroPathing.
 */
public class TeleDriveCommand extends TimeoutCommand {
    private static final String LOG_TAG =
            TeleDriveCommand.class.getSimpleName();
    private final DriveSubsystem drive;
    private final RobotState robotState;
    public Follower follower;
    private RobotStateLocalizer localizer;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier headingSupplier;
    private PathChain pathChain;

    // Primary PIDF Controllers
    private PIDFController translationalPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController drivePIDF;

    private double tolerance;
    private double angleTolerance;
    private double recoveryCounter;

    /**
     * Constructs a new TeleDriveCommand
     *
     * @param drive             The drive subsystem
     * @param translationalPIDF The translational PIDF coefficients
     * @param drivePIDF         The drive PIDF coefficients
     * @param headingPIDF       The heading PIDF coefficients
     * @param xSupplier         The supplier for x
     * @param ySupplier         The supplier for y
     * @param headingSupplier   The supplier for heading
     * @param robotState        The robot state
     * @param tolerance         The tolerance for the distance to the target
     * @param angleTolerance    The tolerance for the angle to the target
     * @param timeout           The timeout for the command
     */
    public TeleDriveCommand(DriveSubsystem drive,
                            CustomPIDFCoefficients translationalPIDF,
                            CustomFilteredPIDFCoefficients drivePIDF,
                            CustomPIDFCoefficients headingPIDF,
                            DoubleSupplier xSupplier,
                            DoubleSupplier ySupplier,
                            DoubleSupplier headingSupplier,
                            RobotState robotState,
                            double tolerance,
                            double angleTolerance,
                            double timeout) {
        super(timeout);
        this.drive = drive;
        this.robotState = robotState;
        localizer = new RobotStateLocalizer(robotState);
        follower = new Follower(localizer);
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.headingSupplier = headingSupplier;
        this.tolerance = tolerance;
        this.angleTolerance = angleTolerance;
        this.translationalPIDF = new PIDFController(translationalPIDF);
        this.headingPIDF = new PIDFController(headingPIDF);
        this.drivePIDF = new FilteredPIDFController(drivePIDF);
        recoveryCounter = 0;
        addRequirements(drive);
    }

    /**
     * Overload constructor, given a point instead of suppliers for x, heading, and y
     *
     * @param drive             The drive subsystem
     * @param translationalPIDF The translational PIDF coefficients
     * @param drivePIDF         The drive PIDF coefficients
     * @param headingPIDF       The heading PIDF coefficients
     * @param targetPosition    The target position
     * @param robotState        The robot state
     * @param tolerance         The tolerance for the distance to the target
     * @param angleTolerance    The tolerance for the angle to the target
     * @param timeout           The timeout for the command
     */
    public TeleDriveCommand(DriveSubsystem drive,
                            CustomPIDFCoefficients translationalPIDF,
                            CustomFilteredPIDFCoefficients drivePIDF,
                            CustomPIDFCoefficients headingPIDF,
                            Waypoint targetPosition,
                            RobotState robotState,
                            double tolerance,
                            double angleTolerance,
                            double timeout) {
        this(drive, translationalPIDF, drivePIDF, headingPIDF,
                targetPosition::getX, targetPosition::getY, targetPosition::getHeading,
                robotState, tolerance, angleTolerance, timeout);
    }


    @Override
    public void initialize() {
        super.initialize();
        recoveryCounter = 0;
        // Makes sure that all primary PIDF coefficients are set
        if (translationalPIDF == null) {
            throw new IllegalArgumentException("Translational PIDF coefficients not set");
        }
        if (headingPIDF == null) {
            throw new IllegalArgumentException("Heading PIDF coefficients not set");
        }
        if (drivePIDF == null) {
            throw new IllegalArgumentException("Drive PIDF coefficients not set");
        }

        // Sets the primary PIDF coefficients
        follower.setTranslationalPIDF(translationalPIDF.getCoefficients());
        follower.setHeadingPIDF(headingPIDF.getCoefficients());
        follower.setDrivePIDF(drivePIDF.getCoefficients());
        follower.disableSecondaryPIDS();

        pathChain = new PathBuilder()
                .addBezierLine(
                        new Point(robotState.getRobotCurrentPose().getX(), robotState.getRobotCurrentPose().getY()),
                        new Point(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                )
                .setLinearHeadingInterpolation(robotState.getRobotCurrentPose().getHeading(), headingSupplier.getAsDouble())
                .build();

        // Finds the final waypoint in the path chain
        Path finalPath = pathChain.getPath(pathChain.size() - 1);
        Waypoint target =
                PoseTranslator.pointToWaypoint(finalPath.getLastControlPoint());
        target = new Waypoint(target.getX(), target.getY(), finalPath.getEndHeading());

        // Sets the robot's final pose to the final waypoint found
        robotState.setRobotFinalPose(target);
        follower.followPath(pathChain, true);
    }

    /**
     * Calculate the distance to the target
     *
     * @param current current waypoint
     * @param target  target waypoint
     * @return the distance to the target
     */
    protected double distToTarget(Waypoint current, Waypoint target) {
        return Math.hypot(target.getX() - current.getX(),
                target.getY() - current.getY());
    }

    /**
     * Calculate the angle distance to the target
     *
     * @param currentHeading current heading
     * @param targetHeading  target heading
     * @return the angle distance to the target
     */
    protected double angleDistance(double currentHeading, double targetHeading) {
        return Math.abs(currentHeading - targetHeading);
    }

    /**
     * Returns whether the robot is stuck. This is calculated by when the
     * robot isn't moving for a period of time
     *
     * @return whether the robot is stuck
     */
    public boolean isRobotStuck() {
        return follower.isRobotStuck();
    }

    @Override
    public void execute() {
        super.execute();
        // If the robot is stuck or the timeout is reached for the first time, we need to recover
        if (isRobotStuck() || (isTimeoutReached() && recoveryCounter == 0)) {
            // Generate a new path chain using the robot's current and final poses
            pathChain = new PathBuilder().addBezierLine(
                    new Point(robotState.getRobotCurrentPose().getX(), robotState.getRobotCurrentPose().getY()),
                    new Point(robotState.getRobotFinalPose().getX(), robotState.getRobotFinalPose().getY())
            ).setLinearHeadingInterpolation(
                    robotState.getRobotCurrentPose().getHeading(),
                    robotState.getRobotFinalPose().getHeading()
            ).build();
            follower = new Follower(localizer);
            follower.setTranslationalPIDF(translationalPIDF.getCoefficients());
            follower.setHeadingPIDF(headingPIDF.getCoefficients());
            follower.setDrivePIDF(drivePIDF.getCoefficients());
            follower.disableSecondaryPIDS();
            follower.followPath(pathChain, true);
            recoveryCounter++;
//            RobotLog.dd(LOG_TAG, "Recovery attempt: %f", recoveryCounter);
        }

        drive.drivePedroPath(follower.getCurrentDriveVectors());
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotCentric(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        if (tolerance < 0 || angleTolerance < 0) {
            throw new IllegalStateException("Tolerance and angle tolerance must be set");
        }

        Waypoint current = robotState.getRobotCurrentPose();
        Waypoint target = robotState.getRobotFinalPose();

//        RobotLog.dd(LOG_TAG, "Current State: %s", robotState.getCurrentAutoState());
//        RobotLog.dd(LOG_TAG, "Distance: %f", distToTarget(current, target));
//        RobotLog.dd(LOG_TAG, "Angular Distance: %f", Math.toDegrees(angleDistance(current.getHeading(), target.getHeading())));

        if (distToTarget(current, target) < tolerance
                && angleDistance(current.getHeading(), target.getHeading()) < angleTolerance) {
            return true;
        }

        return false;
    }
}
