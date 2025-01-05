package org.firstinspires.ftc.teamcode.autostates;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketNoTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.core.paths.Waypoint;

/**
 * Drives to Preload Drop
 */
public class DriveToPreloadDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToPreloadDropState.class.getSimpleName();
    private static final double TOLERANCE = 3;
    private static final double ANGULAR_TOLERANCE = Math.toRadians(5);
    private RobotState robotState;

    /**
     * Constructor for the DriveToPreloadDropState
     *
     * @param name The name of the state
     */
    public DriveToPreloadDropState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState);
        this.robotState = robotState;
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new WaitCommand(250),
                        new DropperHighBasketNoTransferAction(dropper, robotState)
                )
        );
    }

    /**
     * Calculate the distance to the target
     * @param current current waypoint
     * @param target target waypoint
     * @return the distance to the target
     */
    private double distToTarget(Waypoint current, Waypoint target) {
        return Math.hypot(target.getX() - current.getX(),
                target.getY() - current.getY());
    }

    /**
     * Calculate the angle distance to the target
     * @param currentHeading current heading
     * @param targetHeading target heading
     * @return the angle distance to the target
     */
    private double angleDistance(double currentHeading, double targetHeading) {
        return Math.abs(currentHeading - targetHeading);
    }

    /**
     * Check if the robot has reached the target
     * @return the State of the robot from the AutoState enum
     */
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
            return AutoState.SAMPLE_0_DROP_COMPLETE;
        }

        return AutoState.RUNNING;
    }
}
