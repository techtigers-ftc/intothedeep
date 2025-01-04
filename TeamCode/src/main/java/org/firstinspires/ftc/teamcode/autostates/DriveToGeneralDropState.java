package org.firstinspires.ftc.teamcode.autostates;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.core.paths.Waypoint;

/**
 * Drive state that drives the robot to a sample drop from an intake
 */
public class DriveToGeneralDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToGeneralDropState.class.getSimpleName();
    private static final double TOLERANCE = 3;
    private static final double ANGULAR_TOLERANCE = Math.toRadians(5);
    private RobotState robotState;

    /**
     * Constructor for the DriveToGeneralDropState
     *
     * @param name The name of the state
     */
    public DriveToGeneralDropState(String name, DriveSubsystem drive, DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        super(name, drive, robotState);
        this.robotState = robotState;

        addCommands(
                autoDriveCommand,
                new DropperHighBasketAction(dropper, intake, robotState)
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
                && angleDistance(current.getHeading(), target.getHeading()) < ANGULAR_TOLERANCE
                    && robotState.getDropperState() == DropperState.HIGH_BASKET) {
            return AutoState.END_1;
        }

        return AutoState.RUNNING;
    }
}
