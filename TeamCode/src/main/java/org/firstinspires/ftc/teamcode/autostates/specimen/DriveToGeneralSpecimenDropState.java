package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperCarryNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.core.display.Color;

/**
 * Drives to general specimen drop
 */
public class DriveToGeneralSpecimenDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToGeneralSpecimenDropState.class.getSimpleName();

    /**
     * Constructor for the DriveToGeneralSpecimenDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DriveToGeneralSpecimenDropState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState, 3.5);
        addCommands(
                autoDriveCommand,
                new DropperCloseAction(dropper),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new DropperCarryNoTransferAction(dropper, robotState)
                )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperState() == DropperState.FORWARD_CARRY) {
            robotState.setDebugColor(Color.BLUE);
            return AutoState.DRIVE_END;
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            robotState.setDebugColor(Color.GREEN);
            return AutoState.TIMEOUT;
        }
        return AutoState.RUNNING;
    }
}
