package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

public class PedroPathFollowCommand extends CommandBase {
    private final RobotState robotState;
    private final DriveSubsystem driveSubsystem;
    private final Follower follower;
    private PathChain pathChain;
    private Pose finalPose;

    public PedroPathFollowCommand(RobotState robotState, DriveSubsystem drive, Follower follower) {
        this.robotState = robotState;
        this.driveSubsystem = drive;
        this.follower = follower;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        follower.followPath(getPathChain());
    }

    @Override
    public void execute() {
        follower.update();
        driveSubsystem.driveFollower(follower);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setMotorPowers(0, 0, 0, 0);
        follower.breakFollowing();
        follower.resetPIDFToConstantDefault();
    }

    @Override
    public boolean isFinished() {
        return follower.isBusy();
    }

    public PathChain getPathChain() {
        return pathChain;
    }

    public void setPathChain(PathChain pathChain) {
        this.pathChain = pathChain;
    }

    public void setSecondaryTranslationalPIDF(double p, double i, double d) {
        follower.setSecondaryTranslationalPIDF(p, i, d, 0);
    }

    public void setSecondaryTranslationalIntegral(double p, double i, double d) {
        follower.setSecondaryTranslationalIntegral(p, i, d, 0);
    }

    public void setTranslationalPIDF(double p, double i, double d) {
        follower.setTranslationalPIDF(p, i, d, 0);
    }

    public void setTranslationalIntegral(double p, double i, double d) {
        follower.setTranslationalIntegral(p, i, d, 0);
    }

    public void setSecondaryHeadingPIDF(double p, double i, double d) {
        follower.setSecondaryHeadingPIDF(p, i, d, 0);
    }

    public void setHeadingPIDF(double p, double i, double d) {
        follower.setHeadingPIDF(p, i, d, 0);
    }

    public void setSecondaryDrivePIDF(double p, double i, double d, double t) {
        follower.setSecondaryDrivePIDF(p, i, d, t, 0);
    }

    public void setDrivePIDF(double p, double i, double d, double t) {
        follower.setDrivePIDF(p, i, d, t, 0);
    }
}
