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
 * A State to Drive to the intake position
 * Used autoCommand to drive to a custom intake position
 */
public class DriveToIntakeState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToIntakeState.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Constructor for the DriveToIntakeState
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

    @Override
    public AutoState getCurrentCondition() {
        if (getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperState() == DropperState.PRE_TRANSFER) {
            return AutoState.DRIVE_END;
        }

        return AutoState.RUNNING;
    }
}
