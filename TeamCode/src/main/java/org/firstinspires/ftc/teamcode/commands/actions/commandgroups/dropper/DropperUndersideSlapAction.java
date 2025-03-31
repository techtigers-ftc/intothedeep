package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.drive.RawPowerDriveAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that moves the dropper slides in order to hang the specimen on the chamber forwards
 */

public class DropperUndersideSlapAction extends SequentialCommandGroup {
    private static final String LOG_TAG = DropperUndersideSlapAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new DropperSlapAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperUndersideSlapAction(DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new ParallelCommandGroup(
                        new DropperSlidesAbsoluteAction(dropper, 8, 1),
                        new DropperPitchAction(dropper, 155, 0),
                        new SequentialCommandGroup(
                                new RawPowerDriveAction(drive, -1, 0.2)
                        )
                ),
                new DropperOpenAction(dropper)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setDropperState(DropperState.FRONT_SLAP);
        }
    }
}
