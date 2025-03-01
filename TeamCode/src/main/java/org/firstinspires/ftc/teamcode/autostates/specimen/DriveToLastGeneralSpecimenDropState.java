package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryWallAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
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
public class DriveToLastGeneralSpecimenDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToLastGeneralSpecimenDropState.class.getSimpleName();

    /**
     * Constructor for the DriveToGeneralSpecimenDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     * @param intake     The intake subsystem
     */
    public DriveToLastGeneralSpecimenDropState(String name, DriveSubsystem drive, DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        super(name, drive, robotState, 3.5);
        // TODO: delete intake
        addCommands(
                autoDriveCommand,
                new DropperForwardCarryWallAction(dropper, robotState),
                new IntakeReadyToPickupAction(intake, robotState, () -> 0, () -> 90)
                );
    }

    @Override
    public void initialize() {
        super.initialize();
        robotState.setDebugColor(Color.BLACK);
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
