package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * Drive state that drives the robot from the chamber to a sample drop
 */
public class DriveToFirstSpecimenIntakeState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToFirstSpecimenIntakeState.class.getSimpleName();

    /**
     * Constructor for the DriveFromChamberSampleDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DriveToFirstSpecimenIntakeState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState);
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new IntakeTuckAction(intake, robotState),
                        new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getY() < 30 && robotState.getRobotCurrentPose().getHeading() > Math.toRadians(230)),
                        new IntakeOpenAction(intake)
                ),
                new DropperForwardWallIntakeNoTransferAction(dropper, robotState)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperState() == DropperState.WALL_INTAKE &&
                robotState.getDropperClawState() == ClawState.OPEN) {
            return AutoState.DRIVE_END;
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        }
        return AutoState.RUNNING;
    }
}
