package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperCarryNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.core.display.Color;

/**
 * Drives to general specimen drop
 */
public class DriveToLastSpecimenDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToLastSpecimenDropState.class.getSimpleName();

    /**
     * Constructor for the DriveToGeneralSpecimenDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DriveToLastSpecimenDropState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState, 3.5);
        addCommands(
                autoDriveCommand,
                new DropperCarryNoTransferAction(dropper, robotState),
                new DropperCloseAction(dropper),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getX() < 80),
                        new IntakeReadyToPickupAction(intake, robotState, () -> 0)
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
