package org.firstinspires.ftc.teamcode.autostates;

import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.util.FilteredPIDFController;
import org.firstinspires.ftc.teamcode.pedropathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;
import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A base class for autonomous drive states, using a parallel command group.
 */
public abstract class DriveStateBase extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG = DriveStateBase.class.getSimpleName();
    protected final AutoDriveCommand autoDriveCommand;
    private PathChain pathChain;
    private PIDFController translationalPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController drivePIDF;

    /**
     * Constructor for the SequentialCommandGroupState
     *
     * @param name The name of the state
     * @param drive The drive subsystem
     * @param robotState The robot state
     */
    public DriveStateBase(String name, DriveSubsystem drive, RobotState robotState) {
        super(name);
        autoDriveCommand = new AutoDriveCommand(drive, robotState);
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
}
