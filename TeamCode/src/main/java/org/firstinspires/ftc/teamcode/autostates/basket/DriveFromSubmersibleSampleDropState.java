package org.firstinspires.ftc.teamcode.autostates.basket;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * Drive state that drives the robot from the submersible to a sample drop
 */
public class DriveFromSubmersibleSampleDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveFromSubmersibleSampleDropState.class.getSimpleName();
    // TODO: Test this state

    /**
     * Constructor for the DriveFromSubmersibleSampleDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param intake     The intake subsystem
     * @param robotState The robot state
     */
    public DriveFromSubmersibleSampleDropState(String name, DriveSubsystem drive, DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        super(name, drive, robotState);
        addCommands(
                autoDriveCommand,
                new DropperHighBasketAction(dropper, intake, robotState)
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
