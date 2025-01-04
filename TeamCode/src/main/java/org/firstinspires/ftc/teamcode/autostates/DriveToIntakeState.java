package org.firstinspires.ftc.teamcode.autostates;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.core.paths.Waypoint;

/**
 * A test autonomous state that drives the robot using PedroPathing.
 */
public class DriveToIntakeState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToIntakeState.class.getSimpleName();
    private static final double TOLERANCE = 1.5;
    private static final double ANGULAR_TOLERANCE = Math.toRadians(2);
    private final RobotState robotState;

    /**
     * Constructor for the SequentialCommandGroupState
     *
     * @param name The name of the state
     */
    public DriveToIntakeState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState);
        this.robotState = robotState;
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new DropperPreTransferAction(dropper, robotState)
                )
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
                    && robotState.getDropperState() == DropperState.PRE_TRANSFER) {
            return AutoState.END_1;
        }

        return AutoState.RUNNING;
    }
}
