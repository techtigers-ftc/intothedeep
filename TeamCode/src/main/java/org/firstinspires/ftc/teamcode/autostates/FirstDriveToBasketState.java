package org.firstinspires.ftc.teamcode.autostates;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommandBase;
import org.firstinspires.ftc.teamcode.commands.FirstDriveToBasketCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;
import team.techtigers.core.paths.Waypoint;

/**
 * A test autonomous state that drives the robot using PedroPathing.
 */
public class FirstDriveToBasketState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            FirstDriveToBasketState.class.getSimpleName();
    private static final double TOLERANCE = 1;
    private static final double ANGULAR_TOLERANCE = Math.toRadians(5);
    private RobotState robotState;

    /**
     * Constructor for the SequentialCommandGroupState
     *
     * @param name The name of the state
     */
    public FirstDriveToBasketState(String name, DriveSubsystem drive, RobotState robotState) {
        super(name);
        this.robotState = robotState;
        addCommands(
                new FirstDriveToBasketCommand(drive, robotState)
        );
    }

    private double distToTarget(Waypoint current, Waypoint target) {
        return Math.hypot(target.getX() - current.getX(),
                target.getY() - current.getY());
    }

    private double angleDistance(double currentHeading, double targetHeading) {
        return Math.abs(currentHeading - targetHeading);
    }

    @Override
    public AutoState getCurrentCondition() {
        Waypoint current = robotState.getRobotCurrentPose();
        Waypoint target = robotState.getRobotFinalPose();

        RobotLog.dd(LOG_TAG, "Dist to target: %f Angle diff: %f",
                distToTarget(current, target),
                angleDistance(current.getHeading(), target.getHeading()));
        if (distToTarget(current, target) < TOLERANCE
                && angleDistance(current.getHeading(), target.getHeading()) < ANGULAR_TOLERANCE) {
            return AutoState.END_1;
        }

        return AutoState.RUNNING;
    }
}
