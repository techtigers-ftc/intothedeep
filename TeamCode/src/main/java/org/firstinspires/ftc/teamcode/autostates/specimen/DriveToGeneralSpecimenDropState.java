package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackwardCarryAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * Drives to general specimen drop
 */
public class DriveToGeneralSpecimenDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToGeneralSpecimenDropState.class.getSimpleName();

    /**
     * Constructor for the DriveToGeneralSpecimenIntakeState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     * @param intake The intake subsystem
     */
    public DriveToGeneralSpecimenDropState(String name, DriveSubsystem drive, DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        super(name, drive, robotState);
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new IntakeReadyToTransferAction(intake, robotState),
                        new DropperBackwardCarryAction(dropper, intake, robotState)
                )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperState() == DropperState.BACKWARD_CARRY) {
            return AutoState.DRIVE_END;
        }
        return AutoState.RUNNING;
    }
}
