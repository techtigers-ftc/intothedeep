package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A state to drive to the first intake on the specimen auto
 */
public class DriveToFirstPushState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToFirstPushState.class.getSimpleName();

    /**
     * Constructor for a DriveToFirstPush state
     *
     * @param name The name of the state
     * @param drive The drive subsystem
     * @param dropper the dropper subsystem
     * @param robotState The robot state
     */
    public DriveToFirstPushState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState, 5);
        addCommands(
                autoDriveCommand,
                new DropperWallIntakeNoTransferAction(dropper, robotState)
        );
    }
}
