package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.SequentialReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFullReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A state to drive to the park position from the chamber
 */
public class DriveFromChamberToSpecimenPark extends DriveStateBase {
    private static final String LOG_TAG =
            DriveFromChamberToSpecimenPark.class.getSimpleName();

    /**
     * Constructor for the DriveToPark
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param intake     The intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState The robot state
     */
    public DriveFromChamberToSpecimenPark(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState, 5);
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new DropperPreTransferAction(dropper, robotState)
                ),
                // Tuck code but does not open the claw
                new IntakeSlidesAbsoluteAction(intake, () -> 0, 0.75),
                new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_TUCK_POSITION, 200),
                new IntakeClawRotationAction(intake, () -> IntakeSubsystem.CLAW_ROTATION_TUCK_POSITION, 200),
                new IntakeWristRotationAction(intake, IntakeSubsystem.WRIST_ROTATION_TUCK_POSITION, 200)
        );
    }
}
