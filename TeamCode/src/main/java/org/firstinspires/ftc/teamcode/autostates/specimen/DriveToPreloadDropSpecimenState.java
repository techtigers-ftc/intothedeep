package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryNoTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * Drives to the chamber and clips the preload specimen for the beginning of the specimen auto
 */
public class DriveToPreloadDropSpecimenState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToPreloadDropSpecimenState.class.getSimpleName();

    /**
     * Constructor for the DriveToPreloadDropState
     *
     * @param name The name of the state
     * @param drive The drive subsystem
     * @param dropper The dropper subsystem
     * @param robotState The robot state
     */
    public DriveToPreloadDropSpecimenState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState);
        addCommands(
                autoDriveCommand,
                new DropperForwardCarryNoTransferAction(dropper, robotState)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperState() == DropperState.FORWARD_CARRY) {
            return AutoState.DRIVE_END;
        }
        return AutoState.RUNNING;
    }
}
