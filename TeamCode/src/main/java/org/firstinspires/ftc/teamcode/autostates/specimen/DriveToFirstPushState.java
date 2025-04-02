package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperUndersideSlapNoDriveAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.drive.RawPowerDriveAction;
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
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    the dropper subsystem
     * @param robotState The robot state
     */
    public DriveToFirstPushState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState, 5);
        addCommands(
                new SequentialCommandGroup(
                        new WaitCommand(50),
                        new RawPowerDriveAction(drive, -0.8, 0.2),
                        autoDriveCommand
                ),
                new SequentialCommandGroup(
                        new DropperUndersideSlapNoDriveAction(dropper, robotState),
                        new DropperWallIntakeNoTransferAction(dropper, robotState)
                )
        );
    }
}
