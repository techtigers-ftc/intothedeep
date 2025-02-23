package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

/**
 * A State to Drive to the intake position
 * Used autoCommand to drive to a custom intake position
 */
public class DriveToGeneralSampleIntakeState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToGeneralSampleIntakeState.class.getSimpleName();

    /**
     * Constructor for the DriveToGeneralSampleIntakeState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DriveToGeneralSampleIntakeState(String name, DriveSubsystem drive, DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        super(name, drive, robotState, 1.25);
        addCommands(
                autoDriveCommand,
                new InstantCommand(() -> robotState.setIntakeState(IntakeState.READY_TO_PICKUP)),
                new SequentialCommandGroup(
                        new DropperPreTransferAction(dropper, robotState)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new IntakeClawRotationAction(intake, () -> Math.toDegrees(robotState.getRobotFinalPose().getHeading()), 50)
                )
        );
    }
}
