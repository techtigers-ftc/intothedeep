package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
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

    /**
     * Constructor for the DriveToGeneralSubmersibleIntakeState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param intake     The intake subsystem
     * @param robotState The robot state
     */
    public DriveToGeneralSubmersibleIntakeState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState, 3);
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new DropperPreTransferAction(dropper, robotState)
                ),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getY() > 55),
                        new IntakeReadyToPickupAction(intake, robotState, () -> 2)
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
