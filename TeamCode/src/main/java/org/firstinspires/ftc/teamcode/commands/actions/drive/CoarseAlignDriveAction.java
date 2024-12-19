package org.firstinspires.ftc.teamcode.commands.actions.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseTranslator;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;

/**
 * A
 */
public class CoarseAlignDriveAction extends CommandBase {
    private final double tolerance;
    private final DriveSubsystem drive;
    private final RobotState robotState;
    private final Follower follower;
    private double targetPosition;

    public CoarseAlignDriveAction(DriveSubsystem drive, RobotState robotState, double tolerance) {
        this.drive = drive;
        this.robotState = robotState;
        this.tolerance = tolerance;
        follower = new Follower(robotState);
    }

    @Override
    public void initialize() {
        // Set the PIDF coefficients
//        follower.setTranslationalPIDF(1.1, 0, 0.05, 0);
//        follower.setHeadingPIDF(3, 0, 0.05, 0);
//        follower.setDrivePIDF(0.003, 0, 0.00006, 0 ,0);
        follower.setTranslationalPIDF(TuningConstants.translationalP, TuningConstants.translationalI, TuningConstants.translationalD, 0);
        follower.setHeadingPIDF(TuningConstants.headingP, TuningConstants.headingI, TuningConstants.headingD, 0);
        follower.setDrivePIDF(TuningConstants.driveP, TuningConstants.driveI, TuningConstants.driveD, 0, 0);

        Pose currentPose = PoseTranslator.waypointToPose(robotState.getRobotCurrentPose());
        targetPosition = currentPose.getY() - robotState.getBlockLateralCoarse();
        follower.holdPoint(new Point(currentPose.getX(), targetPosition), currentPose.getHeading());
        RobotLog.dd("align", "ExpectedPos: X: %f Y: %f Heading: %f", currentPose.getX(), targetPosition, currentPose.getHeading());
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
