package org.firstinspires.ftc.teamcode.commands.actions.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.localizers.RobotStateLocalizer;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
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
 * A action which uses pedro pathing to hold to a given point
 */
public class HoldPointAction extends CommandBase {
    private static final String LOG_TAG = HoldPointAction.class.getSimpleName();
    private final double tolerance;
    private final double angleTolerance;
    private final DriveSubsystem drive;
    private final RobotState robotState;
    private final Follower follower;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier headingSupplier;

    // Primary PIDF Controllers
    private PIDFController translationalPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController drivePIDF;

    // Secondary PIDF Controllers
    private PIDFController secondaryTranslationalPIDF;
    private PIDFController secondaryHeadingPIDF;
    private FilteredPIDFController secondaryDrivePIDF;

    /**
     * Creates a new HoldPointAction
     *
     * @param drive           the drive subsystem
     * @param robotState      the robot state
     * @param xSupplier       a supplier which gives x values for the target position
     * @param ySupplier       a supplier which gives x values for the target position
     * @param headingSupplier a supplier which gives heading values for the target position
     * @param tolerance       the tolerance for the distance to the target
     * @param angleTolerance  the tolerance for the angle to the target
     */
    public HoldPointAction(DriveSubsystem drive, RobotState robotState, DoubleSupplier xSupplier,
                           DoubleSupplier ySupplier, DoubleSupplier headingSupplier,
                           double tolerance, double angleTolerance) {
        this.drive = drive;
        this.robotState = robotState;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.headingSupplier = headingSupplier;
        this.tolerance = tolerance;
        this.angleTolerance = angleTolerance;
        follower = new Follower(new RobotStateLocalizer(robotState));
    }

    /**
     * Creates a new HoldPointAction (overload constructor)
     *
     * @param drive          the drive subsystem
     * @param robotState     the robot state
     * @param x              the x value for the target
     * @param y              the y value for the target
     * @param heading        the heading value for the target
     * @param tolerance      the tolerance for the distance to the target
     * @param angleTolerance the tolerance for the angle to the target
     */
    public HoldPointAction(DriveSubsystem drive, RobotState robotState, double x,
                           double y, double heading,
                           double tolerance, double angleTolerance) {
        this(drive, robotState, () -> x, () -> y, () -> heading, tolerance, angleTolerance);
    }

    /**
     * Calculate the distance to the target
     *
     * @param current current waypoint
     * @param target  target waypoint
     * @return the distance to the target
     */
    protected double distToTarget(Waypoint current, Waypoint target) {
        return Math.hypot(target.getX() - current.getX(), target.getY() - current.getY());
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

    @Override
    public void initialize() {
        // Set the PIDF coefficients
//        follower.setTranslationalPIDF(new CustomPIDFCoefficients(0.6, 0, 0.02, 0));
//        follower.setHeadingPIDF(new CustomPIDFCoefficients(3, 0, 0.03, 0));
//        follower.setDrivePIDF(new CustomFilteredPIDFCoefficients(0.002, 0, 0.00035, 0.6, 0));

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

        // Sets the secondary PIDF coefficients in the follower only if they have been configured
        // in the command
        if(secondaryTranslationalPIDF != null) {
            follower.setSecondaryTranslationalPIDF(secondaryTranslationalPIDF.getCoefficients());
        }
        if(secondaryHeadingPIDF != null) {
            follower.setSecondaryHeadingPIDF(secondaryHeadingPIDF.getCoefficients());
        }
        if(secondaryDrivePIDF != null) {
            follower.setSecondaryDrivePIDF(secondaryDrivePIDF.getCoefficients());
        }

        Waypoint target = new Waypoint(xSupplier.getAsDouble(), ySupplier.getAsDouble(), headingSupplier.getAsDouble());
        robotState.setRobotFinalPose(target);
        follower.holdPoint(PoseTranslator.waypointToPose(target));
    }

    @Override
    public void execute() {
        // TODO: try to run the hold path in the execute method
        drive.drivePedroPath(follower.getCurrentDriveVectors());
    }

    @Override
    public boolean isFinished() {
        Waypoint current = robotState.getRobotCurrentPose();
        Waypoint target = robotState.getRobotFinalPose();

        RobotLog.dd(LOG_TAG, "Current Position: %s", current.toString());
        RobotLog.dd(LOG_TAG, "Final Position: %s", target.toString());
        RobotLog.dd(LOG_TAG, "Distance to Target: %f", distToTarget(current, target));
        RobotLog.dd(LOG_TAG, "Distance to Angle Target: %f", angleDistance(current.getHeading(), target.getHeading()));


        return distToTarget(current, target) < tolerance && angleDistance(current.getHeading(), target.getHeading()) < angleTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotCentric(0, 0, 0);
    }


    /**
     * Sets the translational PIDF coefficients for the command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param f the feedforward coefficient
     */
    public void setTranslationalPIDF(double p, double i, double d, double f) {
        translationalPIDF = new PIDFController(new CustomPIDFCoefficients(p, i, d, f));
    }

    /**
     * Sets the secondary translational PIDF coefficients for the command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param f the feedforward coefficient
     */
    public void setSecondaryTranslationalPIDF(double p, double i, double d, double f) {
        secondaryTranslationalPIDF = new PIDFController(new CustomPIDFCoefficients(p, i, d, f));
    }

    /**
     * Sets the heading PIDF coefficients for the command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param f the feedforward coefficient
     */
    public void setHeadingPIDF(double p, double i, double d, double f) {
        headingPIDF = new PIDFController(new CustomPIDFCoefficients(p, i, d, f));
    }

    /**
     * Sets the secondary heading PIDF coefficients for the command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param f the feedforward coefficient
     */
    public void setSecondaryHeadingPIDF(double p, double i, double d, double f) {
        secondaryHeadingPIDF = new PIDFController(new CustomPIDFCoefficients(p, i, d, f));
    }

    /**
     * Sets the drive PIDF coefficients for the command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param t the time constant
     * @param f the feedforward coefficient
     */
    public void setDrivePIDF(double p, double i, double d, double t, double f) {
        drivePIDF = new FilteredPIDFController(new CustomFilteredPIDFCoefficients(p, i, d, t, f));
    }

    /**
     * Sets the secondary drive PIDF coefficients for the command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param t the time constant
     * @param f the feedforward coefficient
     */
    public void setSecondaryDrivePIDF(double p, double i, double d, double t, double f) {
        secondaryDrivePIDF = new FilteredPIDFController(new CustomFilteredPIDFCoefficients(p, i, d, t, f));
    }
}
