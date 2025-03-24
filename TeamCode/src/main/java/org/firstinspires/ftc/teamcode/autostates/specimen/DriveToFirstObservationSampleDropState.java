package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;

/**
 * Drives to the last specimen drop, to bring out the intake after the robot reaches past a certain point
 */
public class DriveToFirstObservationSampleDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToFirstObservationSampleDropState.class.getSimpleName();
    /**
     * Constructor for the DriveToLastSpecimenDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     * @param intake     The intake subsystem
     */
    public DriveToFirstObservationSampleDropState(String name, DriveSubsystem drive, DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        super(name, drive, robotState, 3.5);
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new IntakeTuckAction(intake, robotState),
                        new WaitUntilCommand(() -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()) < 55),
                        new IntakeSlidesAbsoluteAction(intake, () -> 16.5, 1),
                        new WaitUntilCommand(() -> intake.getCurrentSlidePositionInches() > 11),
                        new IntakeOpenAction(intake, 0),
                        new WaitCommand(150)
                ),
                new DropperWallIntakeNoTransferAction(dropper, robotState)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getIntakeClawState() == ClawState.OPEN) {
            return AutoState.DRIVE_END;
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        }
        return AutoState.RUNNING;
    }
}
