package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.follower.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.localization.localizers.RobotStateLocalizer;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.FilteredPIDFController;
import org.firstinspires.ftc.teamcode.pedropathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A class for autonomous drive commands that use PedroPathing.
 */
public class AutoEmergencyDriveCommand extends CommandBase {
    private static final String LOG_TAG =
            AutoDriveCommand.class.getSimpleName();
    public final Follower follower;
    private final DriveSubsystem drive;
    private final RobotState robotState;
    // Primary PIDF Controllers
    private PIDFController translationalPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController drivePIDF;

    // Secondary PIDF Controllers
    private PIDFController secondaryTranslationalPIDF;
    private PIDFController secondaryHeadingPIDF;
    private FilteredPIDFController secondaryDrivePIDF;

    /**
     * Constructs a new AutoEmergencyDriveCommand.
     *
     * @param drive      The drive subsystem
     * @param robotState The robot state
     */
    public AutoEmergencyDriveCommand(DriveSubsystem drive,
                                     RobotState robotState) {
        this.drive = drive;
        this.robotState = robotState;
        RobotStateLocalizer localizer = new RobotStateLocalizer(robotState);
        follower = new Follower(localizer);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
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

        // Sets the secondary PIDF coefficients in the follower only if they have been configured
        // in the command
        if (secondaryTranslationalPIDF != null) {
            follower.setSecondaryTranslationalPIDF(secondaryTranslationalPIDF.getCoefficients());
        } else {
            FollowerConstants.useSecondaryTranslationalPID = false;
        }
        if (secondaryHeadingPIDF != null) {
            follower.setSecondaryHeadingPIDF(secondaryHeadingPIDF.getCoefficients());
        } else {
            FollowerConstants.useSecondaryHeadingPID = false;
        }
        if (secondaryDrivePIDF != null) {
            follower.setSecondaryDrivePIDF(secondaryDrivePIDF.getCoefficients());
        } else {
            FollowerConstants.useSecondaryDrivePID = false;
        }

        // Finds the final waypoint in the path chain
        PathChain pathChain = new PathBuilder().addBezierLine(
                        new Point(robotState.getRobotCurrentPose().getX(), robotState.getRobotCurrentPose().getY()),
                        new Point(robotState.getRobotFinalPose().getX(), robotState.getRobotFinalPose().getY())
                )
                .setLinearHeadingInterpolation(robotState.getRobotCurrentPose().getHeading(), robotState.getRobotFinalPose().getHeading())
                .build();

        follower.followPath(pathChain, true);
    }

    @Override
    public void execute() {
        drive.drivePedroPath(follower.getCurrentDriveVectors());
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotCentric(0, 0, 0);
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
