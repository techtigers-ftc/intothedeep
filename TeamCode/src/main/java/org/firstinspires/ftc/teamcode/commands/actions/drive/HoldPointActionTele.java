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
public class HoldPointActionTele extends CommandBase {
    private static final String LOG_TAG = HoldPointActionTele.class.getSimpleName();
    private double tolerance;
    private double angleTolerance;
    private final DriveSubsystem drive;
    private final RobotState robotState;
    private final Follower follower;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier headingSupplier;

    private Pose targetPosition;

    public HoldPointActionTele(DriveSubsystem drive, RobotState robotState, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier headingSupplier, double tolerance, double angleTolerance) {
        this.drive = drive;
        this.robotState = robotState;
        this.follower = new Follower(new RobotStateLocalizer(robotState));
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.headingSupplier = headingSupplier;
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
        follower.setTranslationalPIDF(new CustomPIDFCoefficients(0.6, 0, 0.02, 0));
        follower.setHeadingPIDF(new CustomPIDFCoefficients(3, 0, 0.03, 0));
        follower.setDrivePIDF(new CustomFilteredPIDFCoefficients(0.002, 0, 0.00035, 0.6, 0));

        targetPosition = new Pose(xSupplier.getAsDouble(), ySupplier.getAsDouble(), headingSupplier.getAsDouble());
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
}
