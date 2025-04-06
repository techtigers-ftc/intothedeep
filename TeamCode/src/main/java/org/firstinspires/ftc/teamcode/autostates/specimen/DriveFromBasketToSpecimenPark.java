package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A state to drive and extend the slides to the park position
 */
public class DriveFromBasketToSpecimenPark extends DriveStateBase {
    private static final String LOG_TAG =
            DriveFromBasketToSpecimenPark.class.getSimpleName();

    /**
     * Constructor for the DriveToPark
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param intake     The intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState The robot state
     */
    public DriveFromBasketToSpecimenPark(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState, 5);
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new DropperPreTransferAction(dropper, robotState)
                ),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getX() > 45),
                        new IntakeSlidesAbsoluteAction(intake, () -> IntakeSubsystem.SLIDES_MAX, 2)
                )
        );
    }
}
