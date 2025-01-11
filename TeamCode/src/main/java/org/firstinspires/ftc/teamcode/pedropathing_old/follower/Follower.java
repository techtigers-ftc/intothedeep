package org.firstinspires.ftc.teamcode.pedropathing_old.follower;

import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.drivePIDFFeedForward;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.drivePIDFSwitch;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.forwardZeroPowerAcceleration;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.headingPIDFFeedForward;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.headingPIDFSwitch;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.lateralZeroPowerAcceleration;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.secondaryDrivePIDFFeedForward;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.secondaryHeadingPIDFFeedForward;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.secondaryTranslationalPIDFFeedForward;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.translationalPIDFFeedForward;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.translationalPIDFSwitch;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.useSecondaryDrivePID;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.useSecondaryHeadingPID;
import static org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants.useSecondaryTranslationalPID;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing_old.DriveVectors;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathGeneration.PathCallback;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.Drawing;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.FilteredPIDFController;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.KalmanFilter;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.PIDFController;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.Pose;
import org.firstinspires.ftc.teamcode.utils.PoseTranslator;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import java.util.ArrayList;

/**
 * This is the Follower class. It handles the actual following of the paths and all the on-the-fly
 * calculations that are relevant for movement.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/4/2024
 */
@Config
public class Follower {
    public static boolean drawOnDashboard = true;
    public static boolean useTranslational = true;
    public static boolean useCentripetal = true;
    public static boolean useHeading = true;
    public static boolean useDrive = true;
    private final RobotState robotState;
    private final int BEZIER_CURVE_BINARY_STEP_LIMIT = FollowerConstants.BEZIER_CURVE_BINARY_STEP_LIMIT;
    private final int AVERAGED_VELOCITY_SAMPLE_NUMBER = FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER;
    public double driveError;
    public double headingError;
    public Vector driveVector;
    public Vector headingVector;
    public Vector translationalVector;
    public Vector centripetalVector;
    public Vector correctiveVector;
    public DriveVectors currentDriveVectors;
    private DriveVectorScaler driveVectorScaler;
    private DashboardPoseTracker dashboardPoseTracker;
    private Pose closestPose;
    private Path currentPath;
    private PathChain currentPathChain;
    private int chainIndex;
    private long[] pathStartTimes;
    private boolean followingPathChain;
    private boolean holdingPosition;
    private boolean isBusy;
    private boolean reachedParametricPathEnd;
    private boolean holdPositionAtEnd;
    private boolean teleopDrive;
    //    private final double maxPower = 1;
    private double previousSecondaryTranslationalIntegral;
    private double previousTranslationalIntegral;
    private double holdPointTranslationalScaling = FollowerConstants.holdPointTranslationalScaling;
    private double holdPointHeadingScaling = FollowerConstants.holdPointHeadingScaling;
    private long reachedParametricPathEndTime;
    private double[] teleopDriveValues;
    private ArrayList<Vector> velocities = new ArrayList<>();
    private ArrayList<Vector> accelerations = new ArrayList<>();
    private Vector averageVelocity;
    private Vector averagePreviousVelocity;
    private Vector averageAcceleration;
    private Vector secondaryTranslationalIntegralVector;
    private Vector translationalIntegralVector;
    private Vector teleopDriveVector;
    private Vector teleopHeadingVector;
    private PIDFController secondaryTranslationalPIDF, secondaryTranslationalIntegral, translationalPIDF, translationalIntegral, secondaryHeadingPIDF, headingPIDF;
    private FilteredPIDFController secondaryDrivePIDF, drivePIDF;
    private KalmanFilter driveKalmanFilter = new KalmanFilter(FollowerConstants.driveKalmanFilterParameters);
    private double[] driveErrors;
    private double rawDriveError;
    private double previousRawDriveError;

    /**
     * This creates a new Follower given a HardwareMap.
     *
     * @param robotState robot state to get poses and velocities
     */
    public Follower(RobotState robotState) {
        secondaryTranslationalPIDF = new PIDFController(new CustomPIDFCoefficients(0,0,0,0));
        secondaryTranslationalIntegral = new PIDFController(new CustomPIDFCoefficients(0,0,0,0));
        translationalPIDF = new PIDFController(new CustomPIDFCoefficients(0,0,0,0));
        translationalIntegral = new PIDFController(new CustomPIDFCoefficients(0,0,0,0));
        secondaryHeadingPIDF = new PIDFController(new CustomPIDFCoefficients(0,0,0,0));
        headingPIDF = new PIDFController(new CustomPIDFCoefficients(0,0,0,0));
        secondaryDrivePIDF = new FilteredPIDFController(new CustomFilteredPIDFCoefficients(0,0,0,0,0));
        drivePIDF = new FilteredPIDFController(new CustomFilteredPIDFCoefficients(0,0,0,0,0));
        this.robotState = robotState;
        initialize();
    }

    public void setSecondaryTranslationalPIDF(double p, double i, double d, double f) {
        secondaryTranslationalPIDF.setCoefficients(new CustomPIDFCoefficients(p, i, d, f));
    }

    public void setSecondaryTranslationalIntegral(double p, double i, double d, double f) {
        secondaryTranslationalIntegral.setCoefficients(new CustomPIDFCoefficients(p, i, d, f));
    }

    public void setTranslationalPIDF(double p, double i, double d, double f) {
        translationalPIDF.setCoefficients(new CustomPIDFCoefficients(p, i, d, f));
    }

    public void setTranslationalIntegral(double p, double i, double d, double f) {
        translationalIntegral.setCoefficients(new CustomPIDFCoefficients(p, i, d, f));
    }

    public void setSecondaryHeadingPIDF(double p, double i, double d, double f) {
        secondaryHeadingPIDF.setCoefficients(new CustomPIDFCoefficients(p, i, d, f));
    }

    public void setHeadingPIDF(double p, double i, double d, double f) {
        headingPIDF.setCoefficients(new CustomPIDFCoefficients(p, i, d, f));
    }

    public void setSecondaryDrivePIDF(double p, double i, double d, double t, double f) {
        secondaryDrivePIDF.setCoefficients(new CustomFilteredPIDFCoefficients(p, i, d, t, f));
    }

    public void setDrivePIDF(double p, double i, double d, double t, double f) {
        drivePIDF.setCoefficients(new CustomFilteredPIDFCoefficients(p, i, d, t, f));
    }

    /**
     * This initializes the follower.
     * In this, the DriveVectorScaler and PoseUpdater is instantiated, the drive motors are
     * initialized and their behavior is set, and the variables involved in approximating first and
     * second derivatives for teleop are set.
     */
    public void initialize() {
        driveVectorScaler = new DriveVectorScaler(FollowerConstants.frontLeftVector);
        dashboardPoseTracker = new DashboardPoseTracker(robotState);

        breakFollowing();
    }

    /**
     * This gets a Point from the current Path from a specified t-value.
     *
     * @return returns the Point.
     */
    public Point getPointFromPath(double t) {
        if (currentPath != null) {
            return currentPath.getPoint(t);
        } else {
            return null;
        }
    }

    /**
     * This returns the current pose from the PoseUpdater.
     *
     * @return returns the pose
     */
    public Pose getPose() {
        return null;
    }

    /**
     * This returns the current velocity of the robot as a Vector.
     *
     * @return returns the current velocity as a Vector.
     */
    public Vector getVelocity() {
        return null;
    }

    /**
     * This returns the magnitude of the current velocity. For when you only need the magnitude.
     *
     * @return returns the magnitude of the current velocity.
     */
    public double getVelocityMagnitude() {
        return getVelocity().getMagnitude();
    }

    /**
     * This holds a Point.
     *
     * @param point   the Point to stay at.
     * @param heading the heading to face.
     */
    public void holdPoint(BezierPoint point, double heading) {
        breakFollowing();
        holdingPosition = true;
        isBusy = false;
        followingPathChain = false;
        currentPath = new Path(point);
        currentPath.setConstantHeadingInterpolation(heading);
        closestPose = currentPath.getClosestPoint(getPose(), 1);
    }

    /**
     * This holds a Point.
     *
     * @param point   the Point to stay at.
     * @param heading the heading to face.
     */
    public void holdPoint(Point point, double heading) {
        holdPoint(new BezierPoint(point), heading);
    }

    /**
     * This holds a Point.
     *
     * @param pose the Point (as a Pose) to stay at.
     */
    public void holdPoint(Pose pose) {
        holdPoint(new Point(pose), pose.getHeading());
    }

    /**
     * This follows a Path.
     * This also makes the Follower hold the last Point on the Path.
     *
     * @param path the Path to follow.
     */
    public void followPath(Path path, boolean holdEnd) {
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        isBusy = true;
        followingPathChain = false;
        currentPath = path;
        closestPose = currentPath.getClosestPoint(getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
    }

    /**
     * This follows a Path.
     *
     * @param path the Path to follow.
     */
    public void followPath(Path path) {
        followPath(path, false);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     * This also makes the Follower hold the last Point on the PathChain.
     *
     * @param pathChain the PathChain to follow.
     */
    public void followPath(PathChain pathChain, boolean holdEnd) {
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        pathStartTimes = new long[pathChain.size()];
        pathStartTimes[0] = System.currentTimeMillis();
        isBusy = true;
        followingPathChain = true;
        chainIndex = 0;
        currentPathChain = pathChain;
        currentPath = pathChain.getPath(chainIndex);
        closestPose = currentPath.getClosestPoint(getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     *
     * @param pathChain the PathChain to follow.
     */
    public void followPath(PathChain pathChain) {
        followPath(pathChain, false);
    }

    /**
     * This starts teleop drive control.
     */
    public void startTeleopDrive() {
        breakFollowing();
        teleopDrive = true;
    }

    /**
     * Updates Pose Tracker on the dashboard.
     */
    public void updatePose() {
        if (drawOnDashboard) {
            dashboardPoseTracker.update();
        }
    }

    /**
     * This calls an update to the PoseUpdater, which updates the robot's current position estimate.
     * This also updates all the Follower's PIDFs, which updates the motor powers.
     */
    public void update() {
        updatePose();

        if (!teleopDrive) {
            RobotLog.dd("PedroPathing", "Not in tele drive");
            if (currentPath != null) {
                RobotLog.dd("PedroPathing", "Current Path isn't null");
                if (holdingPosition) {
                    RobotLog.dd("PedroPathing", "Holding pos");
                    closestPose = currentPath.getClosestPoint(getPose(), 1);

                    currentDriveVectors = new DriveVectors(MathFunctions.scalarMultiplyVector(getTranslationalCorrection(), holdPointTranslationalScaling), MathFunctions.scalarMultiplyVector(getHeadingVector(), holdPointHeadingScaling), new Vector(), getPose().getHeading());
                } else {
                    RobotLog.dd("PedroPathing", "Not Holding pos");
                    if (isBusy) {
                        RobotLog.dd("PedroPathing", "Is Busy");
                        closestPose = currentPath.getClosestPoint(getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);

                        if (followingPathChain) updateCallbacks();

                        currentDriveVectors = new DriveVectors(getCorrectiveVector(), getHeadingVector(), getDriveVector(), getPose().getHeading());
                    }
                    if (currentPath.isAtParametricEnd()) {
                        RobotLog.ww("PedroPathing", "At Parametric End");
                        if (followingPathChain && chainIndex < currentPathChain.size() - 1) {
                            RobotLog.ww("PedroPathing", "Not at last path");
                            // Not at last path, keep going
                            breakFollowing();
                            pathStartTimes[chainIndex] = System.currentTimeMillis();
                            isBusy = true;
                            followingPathChain = true;
                            chainIndex++;
                            currentPath = currentPathChain.getPath(chainIndex);
                            closestPose = currentPath.getClosestPoint(getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
                        } else {
                            RobotLog.ww("PedroPathing", "At last path");
                            // At last path, run some end detection stuff
                            // set isBusy to false if at end
                            if (!reachedParametricPathEnd) {
                                RobotLog.ww("PedroPathing", "Reached end");
                                reachedParametricPathEnd = true;
                                reachedParametricPathEndTime = System.currentTimeMillis();
                            }

                            if ((System.currentTimeMillis() - reachedParametricPathEndTime > currentPath.getPathEndTimeoutConstraint()) || (getVelocityMagnitude() < currentPath.getPathEndVelocityConstraint() && MathFunctions.distance(getPose(), closestPose) < currentPath.getPathEndTranslationalConstraint() && MathFunctions.getSmallestAngleDifference(getPose().getHeading(), currentPath.getClosestPointHeadingGoal()) < currentPath.getPathEndHeadingConstraint())) {
                                RobotLog.ww("PedroPathing", "Actually at end");
                                if (holdPositionAtEnd) {
                                    RobotLog.ww("PedroPathing", "Holding position");
                                    holdPositionAtEnd = false;
                                    holdPoint(new BezierPoint(currentPath.getLastControlPoint()), currentPath.getHeadingGoal(1));
                                } else {
                                    RobotLog.ww("PedroPathing", "Breaking follow");
                                    breakFollowing();
                                }
                            }
                        }
                    }
                }
            }
        } else {
            velocities.add(getVelocity());
            velocities.remove(velocities.get(velocities.size() - 1));

            calculateAveragedVelocityAndAcceleration();

            currentDriveVectors = new DriveVectors(getCentripetalForceCorrection(), teleopHeadingVector, teleopDriveVector, getPose().getHeading());
        }
    }

    /**
     * @return The powers to set all the motors to
     */
    public DriveVectors getCurrentDriveVectors() {
        update();
        return currentDriveVectors;
    }

    /**
     * This sets the teleop drive vectors. This defaults to robot centric.
     *
     * @param forwardDrive determines the forward drive vector for the robot in teleop. In field centric
     *                     movement, this is the x-axis.
     * @param lateralDrive determines the lateral drive vector for the robot in teleop. In field centric
     *                     movement, this is the y-axis.
     * @param heading      determines the heading vector for the robot in teleop.
     */
    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading) {
        setTeleOpMovementVectors(forwardDrive, lateralDrive, heading, true);
    }

    /**
     * This sets the teleop drive vectors.
     *
     * @param forwardDrive determines the forward drive vector for the robot in teleop. In field centric
     *                     movement, this is the x-axis.
     * @param lateralDrive determines the lateral drive vector for the robot in teleop. In field centric
     *                     movement, this is the y-axis.
     * @param heading      determines the heading vector for the robot in teleop.
     * @param robotCentric sets if the movement will be field or robot centric
     */
    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric) {
        teleopDriveValues[0] = MathFunctions.clamp(forwardDrive, -1, 1);
        teleopDriveValues[1] = MathFunctions.clamp(lateralDrive, -1, 1);
        teleopDriveValues[2] = MathFunctions.clamp(heading, -1, 1);
        teleopDriveVector.setOrthogonalComponents(teleopDriveValues[0], teleopDriveValues[1]);
        teleopDriveVector.setMagnitude(MathFunctions.clamp(teleopDriveVector.getMagnitude(), 0, 1));

        if (robotCentric) {
            teleopDriveVector.rotateVector(getPose().getHeading());
        }

        teleopHeadingVector.setComponents(teleopDriveValues[2], getPose().getHeading());
    }

    /**
     * This calculates an averaged approximate velocity and acceleration. This is used for a
     * real-time correction of centripetal force, which is used in teleop.
     */
    public void calculateAveragedVelocityAndAcceleration() {
        averageVelocity = new Vector();
        averagePreviousVelocity = new Vector();

        for (int i = 0; i < velocities.size() / 2; i++) {
            averageVelocity = MathFunctions.addVectors(averageVelocity, velocities.get(i));
        }
        averageVelocity = MathFunctions.scalarMultiplyVector(averageVelocity, 1.0 / ((double) velocities.size() / 2));

        for (int i = velocities.size() / 2; i < velocities.size(); i++) {
            averagePreviousVelocity = MathFunctions.addVectors(averagePreviousVelocity, velocities.get(i));
        }
        averagePreviousVelocity = MathFunctions.scalarMultiplyVector(averagePreviousVelocity, 1.0 / ((double) velocities.size() / 2));

        accelerations.add(MathFunctions.subtractVectors(averageVelocity, averagePreviousVelocity));
        accelerations.remove(accelerations.size() - 1);

        averageAcceleration = new Vector();

        for (int i = 0; i < accelerations.size(); i++) {
            averageAcceleration = MathFunctions.addVectors(averageAcceleration, accelerations.get(i));
        }
        averageAcceleration = MathFunctions.scalarMultiplyVector(averageAcceleration, 1.0 / accelerations.size());
    }

    /**
     * This checks if any PathCallbacks should be run right now, and runs them if applicable.
     */
    public void updateCallbacks() {
        for (PathCallback callback : currentPathChain.getCallbacks()) {
            if (!callback.hasBeenRun()) {
                if (callback.getType() == PathCallback.PARAMETRIC) {
                    // parametric call back
                    if (chainIndex == callback.getIndex() && (getCurrentTValue() >= callback.getStartCondition() || MathFunctions.roughlyEquals(getCurrentTValue(), callback.getStartCondition()))) {
                        callback.run();
                    }
                } else {
                    // time based call back
                    if (chainIndex >= callback.getIndex() && System.currentTimeMillis() - pathStartTimes[callback.getIndex()] > callback.getStartCondition()) {
                        callback.run();
                    }

                }
            }
        }
    }

    /**
     * This resets the PIDFs and stops following the current Path.
     */
    public void breakFollowing() {
        teleopDrive = false;
        holdingPosition = false;
        isBusy = false;
        reachedParametricPathEnd = false;
        secondaryDrivePIDF.reset();
        drivePIDF.reset();
        secondaryHeadingPIDF.reset();
        headingPIDF.reset();
        secondaryTranslationalPIDF.reset();
        secondaryTranslationalIntegral.reset();
        secondaryTranslationalIntegralVector = new Vector();
        previousSecondaryTranslationalIntegral = 0;
        translationalPIDF.reset();
        translationalIntegral.reset();
        translationalIntegralVector = new Vector();
        previousTranslationalIntegral = 0;
        driveVector = new Vector();
        headingVector = new Vector();
        translationalVector = new Vector();
        centripetalVector = new Vector();
        correctiveVector = new Vector();
        driveError = 0;
        headingError = 0;
        rawDriveError = 0;
        previousRawDriveError = 0;
        driveErrors = new double[2];
        for (int i = 0; i < driveErrors.length; i++) {
            driveErrors[i] = 0;
        }
        driveKalmanFilter.reset();

        for (int i = 0; i < AVERAGED_VELOCITY_SAMPLE_NUMBER; i++) {
            velocities.add(new Vector());
        }
        for (int i = 0; i < AVERAGED_VELOCITY_SAMPLE_NUMBER / 2; i++) {
            accelerations.add(new Vector());
        }
        calculateAveragedVelocityAndAcceleration();
        teleopDriveValues = new double[3];
        teleopDriveVector = new Vector();
        teleopHeadingVector = new Vector();

        currentDriveVectors = new DriveVectors(new Vector(), new Vector(), new Vector(), 0);
    }

    /**
     * This returns if the Follower is currently following a Path or a PathChain.
     *
     * @return returns if the Follower is busy.
     */
    public boolean isBusy() {
        return isBusy;
    }

    /**
     * This returns a Vector in the direction the robot must go to move along the path. This Vector
     * takes into account the projected position of the robot to calculate how much power is needed.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the drive vector.
     */
    public Vector getDriveVector() {
        if (!useDrive) return new Vector();
        if (followingPathChain && chainIndex < currentPathChain.size() - 1) {
            return new Vector(1, currentPath.getClosestPointTangentVector().getTheta());
        }

        driveError = getDriveVelocityError();

        if (Math.abs(driveError) < drivePIDFSwitch && useSecondaryDrivePID) {
            secondaryDrivePIDF.updateError(driveError);
            driveVector = new Vector(MathFunctions.clamp(secondaryDrivePIDF.runPIDF() + secondaryDrivePIDFFeedForward * MathFunctions.getSign(driveError), -1, 1), currentPath.getClosestPointTangentVector().getTheta());
            return MathFunctions.copyVector(driveVector);
        }

        drivePIDF.updateError(driveError);
        driveVector = new Vector(MathFunctions.clamp(drivePIDF.runPIDF() + drivePIDFFeedForward * MathFunctions.getSign(driveError), -1, 1), currentPath.getClosestPointTangentVector().getTheta());
        return MathFunctions.copyVector(driveVector);
    }

    /**
     * This returns the velocity the robot needs to be at to make it to the end of the Path
     * at some specified deceleration (well technically just some negative acceleration).
     *
     * @return returns the projected velocity.
     */
    public double getDriveVelocityError() {
        double distanceToGoal;
        if (!currentPath.isAtParametricEnd()) {
            distanceToGoal = currentPath.length() * (1 - currentPath.getClosestPointTValue());
        } else {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(getPose().getX() - currentPath.getLastControlPoint().getX(), getPose().getY() - currentPath.getLastControlPoint().getY());
            distanceToGoal = MathFunctions.dotProduct(currentPath.getEndTangent(), offset);
        }

        Vector distanceToGoalVector = MathFunctions.scalarMultiplyVector(MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector()), distanceToGoal);
        Vector velocity = new Vector(MathFunctions.dotProduct(getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta());

        Vector forwardHeadingVector = new Vector(1.0, getPose().getHeading());
        double forwardVelocity = MathFunctions.dotProduct(forwardHeadingVector, velocity);
        double forwardDistanceToGoal = MathFunctions.dotProduct(forwardHeadingVector, distanceToGoalVector);
        double forwardVelocityGoal = MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() * forwardZeroPowerAcceleration * forwardDistanceToGoal));
        double forwardVelocityZeroPowerDecay = forwardVelocity - MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(forwardVelocity, 2) + 2 * forwardZeroPowerAcceleration * forwardDistanceToGoal));

        Vector lateralHeadingVector = new Vector(1.0, getPose().getHeading() - Math.PI / 2);
        double lateralVelocity = MathFunctions.dotProduct(lateralHeadingVector, velocity);
        double lateralDistanceToGoal = MathFunctions.dotProduct(lateralHeadingVector, distanceToGoalVector);
        double lateralVelocityGoal = MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() * lateralZeroPowerAcceleration * lateralDistanceToGoal));
        double lateralVelocityZeroPowerDecay = lateralVelocity - MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(lateralVelocity, 2) + 2 * lateralZeroPowerAcceleration * lateralDistanceToGoal));

        Vector forwardVelocityError = new Vector(forwardVelocityGoal - forwardVelocityZeroPowerDecay - forwardVelocity, forwardHeadingVector.getTheta());
        Vector lateralVelocityError = new Vector(lateralVelocityGoal - lateralVelocityZeroPowerDecay - lateralVelocity, lateralHeadingVector.getTheta());
        Vector velocityErrorVector = MathFunctions.addVectors(forwardVelocityError, lateralVelocityError);

        previousRawDriveError = rawDriveError;
        rawDriveError = velocityErrorVector.getMagnitude() * MathFunctions.getSign(MathFunctions.dotProduct(velocityErrorVector, currentPath.getClosestPointTangentVector()));

        double projection = 2 * driveErrors[1] - driveErrors[0];

        driveKalmanFilter.update(rawDriveError - previousRawDriveError, projection);

        for (int i = 0; i < driveErrors.length - 1; i++) {
            driveErrors[i] = driveErrors[i + 1];
        }
        driveErrors[1] = driveKalmanFilter.getState();

        return driveKalmanFilter.getState();
    }

    /**
     * This returns a Vector in the direction of the robot that contains the heading correction
     * as its magnitude. Positive heading correction turns the robot counter-clockwise, and negative
     * heading correction values turn the robot clockwise. So basically, Pedro Pathing uses a right-
     * handed coordinate system.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the heading vector.
     */
    public Vector getHeadingVector() {
        if (!useHeading) return new Vector();
        headingError = MathFunctions.getTurnDirection(getPose().getHeading(), currentPath.getClosestPointHeadingGoal()) * MathFunctions.getSmallestAngleDifference(getPose().getHeading(), currentPath.getClosestPointHeadingGoal());
        if (Math.abs(headingError) < headingPIDFSwitch && useSecondaryHeadingPID) {
            secondaryHeadingPIDF.updateError(headingError);
            headingVector = new Vector(MathFunctions.clamp(secondaryHeadingPIDF.runPIDF() + secondaryHeadingPIDFFeedForward * MathFunctions.getTurnDirection(getPose().getHeading(), currentPath.getClosestPointHeadingGoal()), -1, 1), getPose().getHeading());
            return MathFunctions.copyVector(headingVector);
        }
        headingPIDF.updateError(headingError);
        headingVector = new Vector(MathFunctions.clamp(headingPIDF.runPIDF() + headingPIDFFeedForward * MathFunctions.getTurnDirection(getPose().getHeading(), currentPath.getClosestPointHeadingGoal()), -1, 1), getPose().getHeading());
        return MathFunctions.copyVector(headingVector);
    }

    /**
     * This returns a combined Vector in the direction the robot must go to correct both translational
     * error as well as centripetal force.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the corrective vector.
     */
    public Vector getCorrectiveVector() {
        Vector centripetal = getCentripetalForceCorrection();
        Vector translational = getTranslationalCorrection();
        Vector corrective = MathFunctions.addVectors(centripetal, translational);

        if (corrective.getMagnitude() > 1) {
            return MathFunctions.addVectors(centripetal, MathFunctions.scalarMultiplyVector(translational, driveVectorScaler.findNormalizingScaling(centripetal, translational)));
        }

        correctiveVector = MathFunctions.copyVector(corrective);

        return corrective;
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only translational
     * error.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the translational correction vector.
     */
    public Vector getTranslationalCorrection() {
        if (!useTranslational) return new Vector();
        Vector translationalVector = new Vector();
        double x = closestPose.getX() - getPose().getX();
        double y = closestPose.getY() - getPose().getY();
        translationalVector.setOrthogonalComponents(x, y);

        if (!(currentPath.isAtParametricEnd() || currentPath.isAtParametricStart())) {
            translationalVector = MathFunctions.subtractVectors(translationalVector, new Vector(MathFunctions.dotProduct(translationalVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));

            secondaryTranslationalIntegralVector = MathFunctions.subtractVectors(secondaryTranslationalIntegralVector, new Vector(MathFunctions.dotProduct(secondaryTranslationalIntegralVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));
            translationalIntegralVector = MathFunctions.subtractVectors(translationalIntegralVector, new Vector(MathFunctions.dotProduct(translationalIntegralVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));
        }

        if (MathFunctions.distance(getPose(), closestPose) < translationalPIDFSwitch && useSecondaryTranslationalPID) {
            secondaryTranslationalIntegral.updateError(translationalVector.getMagnitude());
            secondaryTranslationalIntegralVector = MathFunctions.addVectors(secondaryTranslationalIntegralVector, new Vector(secondaryTranslationalIntegral.runPIDF() - previousSecondaryTranslationalIntegral, translationalVector.getTheta()));
            previousSecondaryTranslationalIntegral = secondaryTranslationalIntegral.runPIDF();

            secondaryTranslationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(secondaryTranslationalPIDF.runPIDF() + secondaryTranslationalPIDFFeedForward);
            translationalVector = MathFunctions.addVectors(translationalVector, secondaryTranslationalIntegralVector);
        } else {
            translationalIntegral.updateError(translationalVector.getMagnitude());
            translationalIntegralVector = MathFunctions.addVectors(translationalIntegralVector, new Vector(translationalIntegral.runPIDF() - previousTranslationalIntegral, translationalVector.getTheta()));
            previousTranslationalIntegral = translationalIntegral.runPIDF();

            translationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(translationalPIDF.runPIDF() + translationalPIDFFeedForward);
            translationalVector = MathFunctions.addVectors(translationalVector, translationalIntegralVector);
        }

        translationalVector.setMagnitude(MathFunctions.clamp(translationalVector.getMagnitude(), 0, 1));

        this.translationalVector = MathFunctions.copyVector(translationalVector);

        return translationalVector;
    }

    /**
     * This returns the raw translational error, or how far off the closest point the robot is.
     *
     * @return This returns the raw translational error as a Vector.
     */
    public Vector getTranslationalError() {
        Vector error = new Vector();
        double x = closestPose.getX() - getPose().getX();
        double y = closestPose.getY() - getPose().getY();
        error.setOrthogonalComponents(x, y);
        return error;
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only centripetal
     * force.
     * <p>
     * Note: This vector is clamped to be between [0, 1] in magnitude.
     *
     * @return returns the centripetal force correction vector.
     */
    public Vector getCentripetalForceCorrection() {
        if (!useCentripetal) return new Vector();
        double curvature;
        if (!teleopDrive) {
            curvature = currentPath.getClosestPointCurvature();
        } else {
            double yPrime = averageVelocity.getYComponent() / averageVelocity.getXComponent();
            double yDoublePrime = averageAcceleration.getYComponent() / averageVelocity.getXComponent();
            curvature = (yDoublePrime) / (Math.pow(Math.sqrt(1 + Math.pow(yPrime, 2)), 3));
        }
        if (Double.isNaN(curvature)) return new Vector();
        centripetalVector = new Vector(MathFunctions.clamp(FollowerConstants.centripetalScaling * FollowerConstants.mass * Math.pow(MathFunctions.dotProduct(getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), 2) * curvature, -1, 1), currentPath.getClosestPointTangentVector().getTheta() + Math.PI / 2 * MathFunctions.getSign(currentPath.getClosestPointNormalVector().getTheta()));
        return centripetalVector;
    }

    /**
     * This returns the closest pose to the robot on the Path the Follower is currently following.
     * This closest pose is calculated through a binary search method with some specified number of
     * steps to search. By default, 10 steps are used, which should be more than enough.
     *
     * @return returns the closest pose.
     */
    public Pose getClosestPose() {
        return closestPose;
    }

    /**
     * This returns whether the follower is at the parametric end of its current Path.
     * The parametric end is determined by if the closest Point t-value is greater than some specified
     * end t-value.
     * If running a PathChain, this returns true only if at parametric end of last Path in the PathChain.
     *
     * @return returns whether the Follower is at the parametric end of its Path.
     */
    public boolean atParametricEnd() {
        if (followingPathChain) {
            if (chainIndex == currentPathChain.size() - 1) return currentPath.isAtParametricEnd();
            return false;
        }
        return currentPath.isAtParametricEnd();
    }

    /**
     * This returns the t value of the closest point on the current Path to the robot
     * In the absence of a current Path, it returns 1.0.
     *
     * @return returns the current t value.
     */
    public double getCurrentTValue() {
        if (isBusy) return currentPath.getClosestPointTValue();
        return 1.0;
    }

    /**
     * This returns the current path number. For following Paths, this will return 0. For PathChains,
     * this will return the current path number. For holding Points, this will also return 0.
     *
     * @return returns the current path number.
     */
    public double getCurrentPathNumber() {
        if (!followingPathChain) return 0;
        return chainIndex;
    }

    /**
     * This returns a new PathBuilder object for easily building PathChains.
     *
     * @return returns a new PathBuilder object.
     */
    public PathBuilder pathBuilder() {
        return new PathBuilder();
    }

    /**
     * This writes out information about the various motion Vectors to the Telemetry specified.
     *
     * @param telemetry this is an instance of Telemetry or the FTC Dashboard telemetry that this
     *                  method will use to output the debug data.
     */
    public void telemetryDebug(MultipleTelemetry telemetry) {
        telemetry.addData("follower busy", isBusy());
        telemetry.addData("heading error", headingError);
        telemetry.addData("heading vector magnitude", headingVector.getMagnitude());
        telemetry.addData("corrective vector magnitude", correctiveVector.getMagnitude());
        telemetry.addData("corrective vector heading", correctiveVector.getTheta());
        telemetry.addData("translational error magnitude", getTranslationalError().getMagnitude());
        telemetry.addData("translational error direction", getTranslationalError().getTheta());
        telemetry.addData("translational vector magnitude", translationalVector.getMagnitude());
        telemetry.addData("translational vector heading", translationalVector.getMagnitude());
        telemetry.addData("centripetal vector magnitude", centripetalVector.getMagnitude());
        telemetry.addData("centripetal vector heading", centripetalVector.getTheta());
        telemetry.addData("drive error", driveError);
        telemetry.addData("drive vector magnitude", driveVector.getMagnitude());
        telemetry.addData("drive vector heading", driveVector.getTheta());
        telemetry.addData("x", getPose().getX());
        telemetry.addData("y", getPose().getY());
        telemetry.addData("heading", getPose().getHeading());
        telemetry.addData("velocity magnitude", getVelocity().getMagnitude());
        telemetry.addData("velocity heading", getVelocity().getTheta());
        driveKalmanFilter.debug(telemetry);
        telemetry.update();
        if (drawOnDashboard) {
            Drawing.drawDebug(this);
        }
    }

    /**
     * This writes out information about the various motion Vectors to the Telemetry specified.
     *
     * @param telemetry this is an instance of Telemetry or the FTC Dashboard telemetry that this
     *                  method will use to output the debug data.
     */
    public void telemetryDebug(Telemetry telemetry) {
        telemetryDebug(new MultipleTelemetry(telemetry));
    }

    /**
     * This returns the current Path the Follower is following. This can be null.
     *
     * @return returns the current Path.
     */
    public Path getCurrentPath() {
        return currentPath;
    }

    /**
     * This returns the pose tracker for the robot to draw on the Dashboard.
     *
     * @return returns the pose tracker
     */
    public DashboardPoseTracker getDashboardPoseTracker() {
        return dashboardPoseTracker;
    }
}
