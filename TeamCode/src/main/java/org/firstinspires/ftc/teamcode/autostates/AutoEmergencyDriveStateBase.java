package org.firstinspires.ftc.teamcode.autostates;

import org.firstinspires.ftc.teamcode.commands.autocommands.AutoEmergencyDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;
import team.techtigers.core.paths.Waypoint;

public class AutoEmergencyDriveStateBase extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG =  AutoEmergencyDriveStateBase.class.getSimpleName();
    protected final AutoEmergencyDriveCommand autoEmergencyDriveCommand;
    protected final RobotState robotState;
    private double tolerance;
    private double angleTolerance;

    /**
     * Constructor for the AutoEmergencyDriveStateBase
     *
     * @param name the name for the state
     * @param drive the drive subsystem
     * @param robotState the robot state
     * @param timeout the timeout for the state in seconds
     */
    public AutoEmergencyDriveStateBase(String name, DriveSubsystem drive, RobotState robotState, double timeout) {
        super(name, timeout);
        autoEmergencyDriveCommand = new AutoEmergencyDriveCommand(drive, robotState);
        this.robotState = robotState;
        tolerance = -1;
        angleTolerance = -1;
    }

    /**
     * Calculate the distance to the target
     *
     * @param current current waypoint
     * @param target  target waypoint
     * @return the distance to the target
     */
    private double distToTarget(Waypoint current, Waypoint target) {
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
    private double angleDistance(double currentHeading, double targetHeading) {
        return Math.abs(currentHeading - targetHeading);
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

    /**
     * Sets the translational PIDF coefficients for the drive command.
     *
     * @param p the proportional coefficient
     * @param i the integral coefficient
     * @param d the derivative coefficient
     * @param f the feedforward coefficient
     */
    public void setTranslationalPIDF(double p, double i, double d, double f) {
        autoEmergencyDriveCommand.setTranslationalPIDF(p, i, d, f);
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
        autoEmergencyDriveCommand.setSecondaryTranslationalPIDF(p, i, d, f);
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
        autoEmergencyDriveCommand.setHeadingPIDF(p, i, d, f);
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
        autoEmergencyDriveCommand.setSecondaryHeadingPIDF(p, i, d, f);
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
        autoEmergencyDriveCommand.setDrivePIDF(p, i, d, t, f);
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
        autoEmergencyDriveCommand.setSecondaryDrivePIDF(p, i, d, t, f);
    }

    /**
     * Sets the primary PIDF coefficients to the values from tuning constants for tuning purposes ONLY
     * Make sure to hard code values in configurators after tuning
     */
    public void setPrimaryPIDSToTuning() {
        autoEmergencyDriveCommand.setTranslationalPIDF(
                TuningConstants.aTranslationalP, 0,
                TuningConstants.bTranslationalD, 0);
        autoEmergencyDriveCommand.setHeadingPIDF(
                TuningConstants.eHeadingP, 0,
                TuningConstants.fHeadingD, 0);
        autoEmergencyDriveCommand.setDrivePIDF(
                TuningConstants.cDriveP, 0,
                TuningConstants.dDriveD, 0.6, 0);
    }

    /**
     * Sets the secondary PIDF coefficients to the values from tuning constants for tuning purposes ONLY
     * Make sure to hard code values in configurators after tuning
     */
    public void setSecondaryPIDSToTuning() {
        autoEmergencyDriveCommand.setSecondaryTranslationalPIDF(
                TuningConstants.gSecondaryTranslationalP, 0,
                TuningConstants.hSecondaryTranslationalD, 0);
        autoEmergencyDriveCommand.setSecondaryHeadingPIDF(
                TuningConstants.kSecondaryHeadingP, 0,
                TuningConstants.lSecondaryHeadingD, 0);
        autoEmergencyDriveCommand.setSecondaryDrivePIDF(
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

//        RobotLog.dd(LOG_TAG, "Current State: %s", robotState.getCurrentAutoState());
//        RobotLog.dd(LOG_TAG, "Distance: %f", distToTarget(current, target));
//        RobotLog.dd(LOG_TAG, "Angular Distance: %f", Math.toDegrees(angleDistance(current.getHeading(), target.getHeading())));

        if (distToTarget(current, target) < tolerance
                && angleDistance(current.getHeading(), target.getHeading()) < angleTolerance) {
            return AutoState.DRIVE_END;
        }

        if (autoEmergencyDriveCommand.isRobotStuck()) {
            return AutoState.DRIVE_END;
        }

        return AutoState.RUNNING;
    }
}
