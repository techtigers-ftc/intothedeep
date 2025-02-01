package org.firstinspires.ftc.teamcode.commands.actions.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.localizers.RobotStateLocalizer;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.FilteredPIDFController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseTranslator;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;

import java.util.function.DoubleSupplier;

import team.techtigers.core.paths.Waypoint;

/**
 * A action which uses pedro pathing to hold to a given point
 */
@Config
public class TeleHoldPointAction extends CommandBase {
    private static final String LOG_TAG = TeleHoldPointAction.class.getSimpleName();
    public static double TIMEOUT = 10;
    private final double tolerance;
    private final double angleTolerance;
    private final DriveSubsystem drive;
    private final RobotState robotState;
    private final Follower follower;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier headingSupplier;
    private final ElapsedTime timer;

    /**
     * Creates a new HoldPointAction
     *
     * @param drive           the drive subsystem
     * @param robotState      the robot state
     * @param xSupplier       a supplier which gives x values for the target position
     * @param ySupplier       a supplier which gives x values for the target position
     * @param headingSupplier a supplier which gives heading values for the target position
     * @param tolerance       the tolerance for the distance to the target
     * @param angleTolerance  the tolerance for the angle to the target
     */
    public TeleHoldPointAction(DriveSubsystem drive, RobotState robotState,
                            DoubleSupplier xSupplier,
                           DoubleSupplier ySupplier, DoubleSupplier headingSupplier,
                           double tolerance, double angleTolerance) {
        this.drive = drive;
        this.robotState = robotState;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.headingSupplier = headingSupplier;
        this.tolerance = tolerance;
        this.angleTolerance = angleTolerance;
        follower = new Follower(new RobotStateLocalizer(robotState));
        timer = new ElapsedTime();
    }

    /**
     * Creates a new HoldPointAction (overload constructor)
     *
     * @param drive          the drive subsystem
     * @param robotState     the robot state
     * @param x              the x value for the target
     * @param y              the y value for the target
     * @param heading        the heading value for the target
     * @param tolerance      the tolerance for the distance to the target
     * @param angleTolerance the tolerance for the angle to the target
     */
    public TeleHoldPointAction(DriveSubsystem drive, RobotState robotState,
                            double x,
                           double y, double heading,
                           double tolerance, double angleTolerance) {
        this(drive, robotState, () -> x, () -> y, () -> heading, tolerance, angleTolerance);
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
        return Math.abs(currentHeading - targetHeading) % Math.toRadians(360);
    }

    @Override
    public void initialize() {
        // Set the PIDF coefficients
//        follower.setTranslationalPIDF(new CustomPIDFCoefficients(0.43, 0, 0.05,
//                0));
//        follower.setDrivePIDF(new CustomFilteredPIDFCoefficients(0.045, 0, 0.001, 0.6, 0));
//        follower.setHeadingPIDF(new CustomPIDFCoefficients(3, 0, 0.06, 0.1));

        follower.setTranslationalPIDF(new CustomPIDFCoefficients(TuningConstants.aTranslationalP, 0, TuningConstants.bTranslationalD, 0));
        follower.setDrivePIDF(new CustomFilteredPIDFCoefficients(TuningConstants.cDriveP, 0, TuningConstants.dDriveD, 0.6, 0));
        follower.setHeadingPIDF(new CustomPIDFCoefficients(TuningConstants.eHeadingP, 0, TuningConstants.fHeadingD, 0));
//        follower.setSecondaryTranslationalPIDF(new CustomPIDFCoefficients(TuningConstants.gSecondaryTranslationalP, 0, TuningConstants.hSecondaryTranslationalD, 0));
//        follower.setSecondaryDrivePIDF(new CustomFilteredPIDFCoefficients(TuningConstants.iSecondaryDriveP, 0, TuningConstants.jSecondaryDriveD, 0.6, 0));
//        follower.setSecondaryHeadingPIDF(new CustomPIDFCoefficients(TuningConstants.kSecondaryHeadingP, 0, TuningConstants.lSecondaryHeadingD, 0));



        Waypoint target = new Waypoint(xSupplier.getAsDouble(), ySupplier.getAsDouble(), headingSupplier.getAsDouble());
        robotState.setRobotFinalPose(target);
        follower.holdPoint(PoseTranslator.waypointToPose(target));
        timer.reset();
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
        RobotLog.dd(LOG_TAG, "Tolerance: %f, Angle tolerance: %f",
                tolerance, angleTolerance);


        return
                (distToTarget(current, target) < tolerance
                        && angleDistance(current.getHeading(), target.getHeading()) < angleTolerance)
                || timer.seconds() > TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotCentric(0, 0, 0);
    }
}