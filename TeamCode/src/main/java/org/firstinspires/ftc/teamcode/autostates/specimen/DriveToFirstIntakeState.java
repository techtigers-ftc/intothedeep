package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A state to drive to the first intake on the specimen auto
 */
public class DriveToFirstIntakeState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToFirstIntakeState.class.getSimpleName();

    /**
     * Constructor for a DriveToFirstIntake state
     *
     * @param name The name of the state
     * @param drive The drive subsystem
     * @param dropper the dropper subsystem
     * @param robotState The robot state
     */
    public DriveToFirstIntakeState(String name, DriveSubsystem drive, DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        super(name, drive, robotState, 5);
        addCommands(
                autoDriveCommand,
                new DropperWallIntakeAction(dropper, intake, robotState)
        );
    }
}
