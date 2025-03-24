package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A State to Drive to the submersible to intake
 * Used autoCommand to drive to a custom intake position
 */
public class DriveToGeneralSubmersibleIntakeState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToGeneralSubmersibleIntakeState.class.getSimpleName();
    private static final long DELAY_FOR_INTAKE = 200;

    /**
     * Constructor for the DriveToGeneralSubmersibleIntakeState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param intake     The intake subsystem
     * @param robotState The robot state
     */
    public DriveToGeneralSubmersibleIntakeState(String name, DriveSubsystem drive, IntakeSubsystem intake, RobotState robotState) {
        super(name, drive, robotState, 3);
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new WaitCommand(DELAY_FOR_INTAKE),
                        new IntakePrepareToPickupAction(intake, robotState)
                )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperState() == DropperState.PRE_TRANSFER) {
            return AutoState.DRIVE_END;
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        }
        return AutoState.RUNNING;
    }
}
