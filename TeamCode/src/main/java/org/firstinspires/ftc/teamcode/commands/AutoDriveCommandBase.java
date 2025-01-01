package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.FilteredPIDFController;
import org.firstinspires.ftc.teamcode.pedropathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A base class for autonomous drive commands that use PedroPathing.
 */
public abstract class AutoDriveCommandBase extends CommandBase {
    protected final DriveSubsystem drive;
    protected final RobotState robotState;
    protected final Follower follower;
    protected PathChain pathChain;
    protected PIDFController translationalPIDF;
    protected PIDFController headingPIDF;
    protected FilteredPIDFController drivePIDF;

    /**
     * Constructs a new AutoDriveCommandBase.
     *
     * @param drive      The drive subsystem
     * @param robotState The robot state
     */
    public AutoDriveCommandBase(DriveSubsystem drive,
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

        follower.followPath(pathChain);
    }

    @Override
    public void execute() {
        drive.drivePedroPath(follower.getCurrentDriveVectors());
    }

    @Override
    public boolean isFinished() {
        return pathChain.getPath(pathChain.size()-1).isAtParametricEnd();
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotCentric(0,0,0);
    }

    protected void setPathChain(PathChain pathChain) {
        this.pathChain = pathChain;
    }

    protected void setTranslationalPIDF(double p, double i, double d, double f) {
        translationalPIDF = new PIDFController(new CustomPIDFCoefficients(p, i, d, f));
    }

    protected void setHeadingPIDF(double p, double i, double d, double f) {
        headingPIDF = new PIDFController(new CustomPIDFCoefficients(p, i, d, f));
    }

    protected void setDrivePIDF(double p, double i, double d, double t, double f) {
        drivePIDF = new FilteredPIDFController(new CustomFilteredPIDFCoefficients(p, i, d, t, f));
    }
}
