package org.firstinspires.ftc.teamcode.autostates;

import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;
import team.techtigers.core.paths.Waypoint;

/**
 * A base class for autonomous drive states, using a parallel command group.
 */
public abstract class DriveStateBase extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG = DriveStateBase.class.getSimpleName();
    protected final AutoDriveCommand autoDriveCommand;
    protected final RobotState robotState;
    private double tolerance;
    private double angleTolerance;

    /**
     * Constructor for the SequentialCommandGroupState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param robotState The robot state
     * @param timeout    time limit of the drive in seconds
     */
    public DriveStateBase(String name, DriveSubsystem drive, RobotState robotState, double timeout) {
        super(name, timeout);
        this.robotState = robotState;
        autoDriveCommand = new AutoDriveCommand(drive, robotState);
        tolerance = -1;
        angleTolerance = -1;
    }

    /**
     * Overload Constructor without Timeout
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param robotState The robot state
     */
    public DriveStateBase(String name, DriveSubsystem drive, RobotState robotState) {
        this(name, drive, robotState, -1);
    }

    /**
     * Calculate the distance to the target
     *
     * @param current current waypoint
     * @param target  target waypoint
     * @return the distance to the target
     */
    protected double distToTarget(Waypoint current, Waypoint target) {
        return Math.hypot(target.getX() - current.getX(),
                target.getY() - current.getY());
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

    /**
     * Sets the path chain for the drive command.
     *
     * @param pathChain the path chain to run
     */
    public void setPathChain(PathChain pathChain) {
        autoDriveCommand.setPathChain(pathChain);
    }

    /**
     * Sets the translational PIDF coefficients for the drive command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param f the feedforward coefficient
     */
    public void setTranslationalPIDF(double p, double i, double d, double f) {
        autoDriveCommand.setTranslationalPIDF(p, i, d, f);
    }

    /**
     * Sets the secondary translational PIDF coefficients for the drive command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param f the feedforward coefficient
     */
    public void setSecondaryTranslationalPIDF(double p, double i, double d, double f) {
        autoDriveCommand.setSecondaryTranslationalPIDF(p, i, d, f);
    }

    /**
     * Sets the heading PIDF coefficients for the drive command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param f the feedforward coefficient
     */
    public void setHeadingPIDF(double p, double i, double d, double f) {
        autoDriveCommand.setHeadingPIDF(p, i, d, f);
    }

    /**
     * Sets the secondary heading PIDF coefficients for the drive command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param f the feedforward coefficient
     */
    public void setSecondaryHeadingPIDF(double p, double i, double d, double f) {
        autoDriveCommand.setSecondaryHeadingPIDF(p, i, d, f);
    }

    /**
     * Sets the drive PIDF coefficients for the drive command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param t the time constant
     * @param f the feedforward coefficient
     */
    public void setDrivePIDF(double p, double i, double d, double t, double f) {
        autoDriveCommand.setDrivePIDF(p, i, d, t, f);
    }

    /**
     * Sets the secondary drive PIDF coefficients for the drive command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param t the time constant
     * @param f the feedforward coefficient
     */
    public void setSecondaryDrivePIDF(double p, double i, double d, double t, double f) {
        autoDriveCommand.setSecondaryDrivePIDF(p, i, d, t, f);
    }

    /**
     * Sets the primary PIDF coefficients to the values from tuning constants for tuning purposes ONLY
     * Make sure to hard code values in configurators after tuning
     */
    public void setPrimaryPIDSToTuning() {
        autoDriveCommand.setTranslationalPIDF(
                TuningConstants.aTranslationalP, 0,
                TuningConstants.bTranslationalD, 0);
        autoDriveCommand.setHeadingPIDF(
                TuningConstants.eHeadingP, 0,
                TuningConstants.fHeadingD, 0);
        autoDriveCommand.setDrivePIDF(
                TuningConstants.cDriveP, 0,
                TuningConstants.dDriveD, 0.6, 0);
    }

    /**
     * Sets the secondary PIDF coefficients to the values from tuning constants for tuning purposes ONLY
     * Make sure to hard code values in configurators after tuning
     */
    public void setSecondaryPIDSToTuning() {
        autoDriveCommand.setSecondaryTranslationalPIDF(
                TuningConstants.gSecondaryTranslationalP, 0,
                TuningConstants.hSecondaryTranslationalD, 0);
        autoDriveCommand.setSecondaryHeadingPIDF(
                TuningConstants.kSecondaryHeadingP, 0,
                TuningConstants.lSecondaryHeadingD, 0);
        autoDriveCommand.setSecondaryDrivePIDF(
                TuningConstants.iSecondaryDriveP, 0,
                TuningConstants.jSecondaryDriveD, 0.6, 0);
    }

    /**
     * Sets the primary and secondary PIDF coefficients to the defaults from follower constants for tuning purposes ONLY
     * Make sure to hard code values in configurators after tuning
     */
    public void setPIDSToTuning() {
        setPrimaryPIDSToTuning();
        setSecondaryPIDSToTuning();
    }

    /**
     * Sets the tolerance for the drive state
     *
     * @param tolerance the tolerance for the drive state
     */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * Sets the angle tolerance for the drive state
     *
     * @param angleTolerance the angle tolerance for the drive state
     */
    public void setAngleTolerance(double angleTolerance) {
        this.angleTolerance = angleTolerance;
    }

    @Override
    public AutoState getCurrentCondition() {
        if (tolerance < 0 || angleTolerance < 0) {
            throw new IllegalStateException("Tolerance and angle tolerance must be set");
        }

        if (super.isTimeoutReached()) {
            return AutoState.TIMEOUT;
        }

        Waypoint current = robotState.getRobotCurrentPose();
        Waypoint target = robotState.getRobotFinalPose();

        if (distToTarget(current, target) < tolerance
                && angleDistance(current.getHeading(), target.getHeading()) < angleTolerance) {
            return AutoState.DRIVE_END;
        }

        if (autoDriveCommand.isRobotStuck()) {
            return AutoState.DRIVE_END;
        }

        return AutoState.RUNNING;
    }
}
