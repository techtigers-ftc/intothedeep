package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperFrontSlapAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * Drives to Preload Drop
 */
public class DriveToPreloadDropStateSpecimen extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToPreloadDropStateSpecimen.class.getSimpleName();

    /**
     * Constructor for the DriveToPreloadDropState
     *
     * @param name The name of the state
     * @param drive The drive subsystem
     * @param dropper The dropper subsystem
     * @param robotState The robot state
     */
    public DriveToPreloadDropStateSpecimen(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState);
        addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                autoDriveCommand,
                                new DropperForwardCarryNoTransferAction(dropper, robotState)
                        ),
                        new DropperFrontSlapAction(dropper, robotState)
                )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperState() == DropperState.FRONT_SLAP) {
            return AutoState.DRIVE_END;
        }

        return AutoState.RUNNING;
    }
}
