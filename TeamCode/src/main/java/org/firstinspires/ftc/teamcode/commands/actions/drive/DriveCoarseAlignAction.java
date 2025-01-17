package org.firstinspires.ftc.teamcode.commands.actions.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.follower.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.localization.localizers.RobotStateLocalizer;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseTranslator;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A
 */
public class DriveCoarseAlignAction extends CommandBase {
    private final double tolerance;
    private final DriveSubsystem drive;
    private final RobotState robotState;
    private final Follower follower;
    private double targetPosition;

    public DriveCoarseAlignAction(DriveSubsystem drive, RobotState robotState, double tolerance) {
        this.drive = drive;
        this.robotState = robotState;
        this.tolerance = tolerance;
        follower = new Follower(new RobotStateLocalizer(robotState));
    }

    @Override
    public void initialize() {
        // Set the PIDF coefficients
        follower.setTranslationalPIDF(new CustomPIDFCoefficients(0.3, 0, 0.02, 0));
        follower.setHeadingPIDF(new CustomPIDFCoefficients(1, 0, 0.03, 0));
        follower.setDrivePIDF(new CustomFilteredPIDFCoefficients(0.002, 0, 0.00035, 0.6 ,0));

//        follower.setTranslationalPIDF(new CustomPIDFCoefficients(FollowerConstants.translationalPIDFCoefficients.P, FollowerConstants.translationalPIDFCoefficients.I, FollowerConstants.translationalPIDFCoefficients.D, FollowerConstants.translationalPIDFCoefficients.F));
//        follower.setHeadingPIDF(new CustomPIDFCoefficients(FollowerConstants.headingPIDFCoefficients.P, FollowerConstants.headingPIDFCoefficients.I, FollowerConstants.headingPIDFCoefficients.D, FollowerConstants.headingPIDFCoefficients.F));
//        follower.setDrivePIDF(new CustomFilteredPIDFCoefficients(FollowerConstants.drivePIDFCoefficients.P, FollowerConstants.drivePIDFCoefficients.I, FollowerConstants.drivePIDFCoefficients.D, FollowerConstants.drivePIDFCoefficients.T, FollowerConstants.drivePIDFCoefficients.F));

        Pose currentPose = PoseTranslator.waypointToPose(robotState.getRobotCurrentPose());
        targetPosition = currentPose.getY() - robotState.getBlockLateralCoarse();
        follower.holdPoint(new Point(currentPose.getX(), targetPosition), 0);
        //TODO:Tune this heading value later to the one we want
        RobotLog.dd("align", "ExpectedPos: X: %f Y: %f Heading: %f", currentPose.getX(), targetPosition, 0.0);
    }

    @Override
    public void execute() {
        // TODO: add for the edge case where the block is moved by an outside force
        //maybe track the difference in the limelight lateral value
        drive.drivePedroPath(follower.getCurrentDriveVectors());
        RobotLog.dd("align", "CurrentPos: X: %f Y: %f Heading: %f", follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetPosition - robotState.getRobotCurrentPose().getY()) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotCentric(0,0,0);
    }
}
