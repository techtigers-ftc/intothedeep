package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedropathing_old.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.FilteredPIDFController;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.PIDFController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseTranslator;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.paths.Waypoint;

/**
 * A class for autonomous drive commands that use PedroPathing.
 */
public class AutoDriveCommand extends CommandBase {
    private static final String LOG_TAG =
            AutoDriveCommand.class.getSimpleName();

    private final DriveSubsystem drive;
    private final RobotState robotState;
    private final Follower follower;
    private PathChain pathChain;
    private PIDFController translationalPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController drivePIDF;

    /**
     * Constructs a new AutoDriveCommand.
     *
     * @param drive      The drive subsystem
     * @param robotState The robot state
     */
    public AutoDriveCommand(DriveSubsystem drive,
                            RobotState robotState) {
        this.drive = drive;
        this.robotState = robotState;
        follower = new Follower(robotState);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        if (translationalPIDF == null) {
            throw new IllegalArgumentException("Translational PIDF coefficients not set");
        }
        if (headingPIDF == null) {
            throw new IllegalArgumentException("Heading PIDF coefficients not set");
        }
        if (drivePIDF == null) {
            throw new IllegalArgumentException("Drive PIDF coefficients not set");
        }
        if (pathChain == null) {
            throw new IllegalArgumentException("Path chain not set");
        }
        follower.setTranslationalPIDF(translationalPIDF.P(), translationalPIDF.I(), translationalPIDF.D(), translationalPIDF.F());
        follower.setHeadingPIDF(headingPIDF.P(), headingPIDF.I(), headingPIDF.D(), headingPIDF.F());
        follower.setDrivePIDF(drivePIDF.P(), drivePIDF.I(), drivePIDF.D(), drivePIDF.T(), drivePIDF.F());

        Path finalPath = pathChain.getPath(pathChain.size()-1);
        Waypoint target =
                PoseTranslator.pointToWaypoint(finalPath.getLastControlPoint());
        target = new Waypoint(target.getX(), target.getY(), finalPath.getEndHeading());

        robotState.setRobotFinalPose(target);
        follower.followPath(pathChain, true);
    }

    @Override
    public void execute() {
        drive.drivePedroPath(follower.getCurrentDriveVectors());
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotCentric(0,0,0);
    }

    /**
     * Sets the path chain for the command.
     *
     * @param pathChain the path chain to run
     */
    public void setPathChain(PathChain pathChain) {
        this.pathChain = pathChain;
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
}
