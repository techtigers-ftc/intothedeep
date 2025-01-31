package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketNoTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * Drives to Preload Drop
 */
public class DriveToPreloadDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToPreloadDropState.class.getSimpleName();

    /**
     * Constructor for the DriveToPreloadDropState
     *
     * @param name The name of the state
     * @param drive The drive subsystem
     * @param dropper The dropper subsystem
     * @param robotState The robot state
     */
    public DriveToPreloadDropState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState, 5);
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new WaitCommand(250),
                        new DropperHighBasketNoTransferAction(dropper, robotState)
                )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperState() == DropperState.HIGH_BASKET) {
            return AutoState.DRIVE_END;
        } else if(super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        }

        return AutoState.RUNNING;
    }
}
