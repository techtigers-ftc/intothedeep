package org.firstinspires.ftc.teamcode.commands.actions.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.localization.localizers.RobotStateLocalizer;
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
public class AutoHoldPointCommand extends CommandBase {
    private static final String LOG_TAG = AutoHoldPointCommand.class.getSimpleName();
    private final DriveSubsystem drive;
    private final RobotState robotState;
    private final Follower follower;

    // Primary PIDF Controllers
    private PIDFController translationalPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController drivePIDF;

    // Secondary PIDF Controllers
    private PIDFController secondaryTranslationalPIDF;
    private PIDFController secondaryHeadingPIDF;
    private FilteredPIDFController secondaryDrivePIDF;

    private Pose targetPosition;

    /**
     * Creates a new AutoHoldPointCommand
     *
     * @param drive           the drive subsystem
     * @param robotState      the robot state
     */
    public AutoHoldPointCommand(DriveSubsystem drive, RobotState robotState) {
        this.drive = drive;
        this.robotState = robotState;
        follower = new Follower(new RobotStateLocalizer(robotState));
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
        if (targetPosition == null) {
            throw new IllegalArgumentException("Target position not set");
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

        Waypoint target = PoseTranslator.poseToWaypoint(targetPosition);
        robotState.setRobotFinalPose(target);
        follower.holdPoint(targetPosition);
    }

    @Override
    public void execute() {
        // TODO: try to run the hold path in the execute method
        drive.drivePedroPath(follower.getCurrentDriveVectors());
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotCentric(0, 0, 0);
    }

    /**
     * Sets the target position for the robot to hold to
     *
     * @param x the x position
     * @param y the y position
     * @param heading the heading
     */
    public void setTargetPosition(DoubleSupplier x, DoubleSupplier y, DoubleSupplier heading) {
        targetPosition = new Pose(x.getAsDouble(), y.getAsDouble(), heading.getAsDouble());
    }

    /**
     * Sets the target position for the robot to hold to (overload constructor)
     *
     * @param x the x position
     * @param y the y position
     * @param heading the heading
     */
    public void setTargetPosition(double x, double y, double heading) {
        setTargetPosition(() -> x, () -> y, () -> heading);
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
