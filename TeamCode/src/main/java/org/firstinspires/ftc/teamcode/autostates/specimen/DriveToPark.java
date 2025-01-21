package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import java.util.function.DoubleSupplier;

/**
 * A state to drive and extend the slides to the park position
 */
public class DriveToPark extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToPark.class.getSimpleName();

    /**
     * Constructor for the DriveToPark
     *
     * @param name       The name of the state
     * @param intake     The intake subsystem
     * @param drive      The drive subsystem
     * @param dropper    the dropper subsystem
     * @param robotState The robot state
     */
    public DriveToPark(String name, IntakeSubsystem intake, DriveSubsystem drive, DropperSubsystem dropper, DoubleSupplier targetSlidePos, RobotState robotState) {
        super(name, drive, robotState);
        addCommands(
                autoDriveCommand,
                new IntakeSlidesAbsoluteAction(intake, targetSlidePos, 0.5),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new DropperPreTransferAction(dropper, robotState)
                )
        );
    }
}
