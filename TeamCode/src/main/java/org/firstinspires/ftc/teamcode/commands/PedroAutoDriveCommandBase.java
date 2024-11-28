package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.FilteredPIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

public class PedroAutoDriveCommandBase extends CommandBase {
    protected final DriveSubsystem drive;
    protected final RobotState robotState;
    protected final Follower follower;
    protected PathChain pathChain;
    protected PIDFController translationalPIDF;
    protected PIDFController headingPIDF;
    protected FilteredPIDFController drivePIDF;

    public PedroAutoDriveCommandBase(DriveSubsystem drive, RobotState robotState) {
        this.drive = drive;
        this.robotState = robotState;
        addRequirements(drive);
        follower = new Follower(robotState);
    }

    @Override
    public void initialize() {
        follower.setTranslationalPIDF(translationalPIDF.P(), translationalPIDF.I(), translationalPIDF.D(), translationalPIDF.F());
        follower.setHeadingPIDF(headingPIDF.P(), headingPIDF.I(), headingPIDF.D(), headingPIDF.F());
        follower.setDrivePIDF(drivePIDF.P(), drivePIDF.I(), drivePIDF.D(), drivePIDF.T(), drivePIDF.F());

        follower.followPath(pathChain);
    }

    @Override
    public void execute() {
        drive.drivePedroPath(follower.getCurrentDriveVectors());
    }

    protected PathChain getPathChain() {
        return pathChain;
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
