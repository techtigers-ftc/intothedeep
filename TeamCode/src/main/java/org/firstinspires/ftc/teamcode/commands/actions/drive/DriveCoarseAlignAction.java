package org.firstinspires.ftc.teamcode.commands.actions.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Pose;
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
        follower = new Follower(robotState);
    }

    @Override
    public void initialize() {
        // Set the PIDF coefficients
        follower.setTranslationalPIDF(0.5, 0, 0.05, 0);
        follower.setHeadingPIDF(3, 0, 0.05, 0);
        follower.setDrivePIDF(0.003, 0, 0.00006, 0 ,0);

        Pose currentPose = PoseTranslator.waypointToPose(robotState.getRobotCurrentPose());
        targetPosition = currentPose.getY() - robotState.getBlockLateralCoarse();
        follower.holdPoint(new Point(currentPose.getX(), targetPosition), currentPose.getHeading());
    }

    @Override
    public void execute() {
        // TODO: add for the edge case where the block is moved by an outside force
        //maybe track the difference in the limelight lateral value
        drive.drivePedroPath(follower.getCurrentDriveVectors());
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
