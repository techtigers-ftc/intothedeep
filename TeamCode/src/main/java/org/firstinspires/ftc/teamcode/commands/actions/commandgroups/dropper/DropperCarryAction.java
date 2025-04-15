package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAfterTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.TransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper to the forward high chamber drop position, with the specimen
 * upside down, ready to be clipped downwards onto the high chamber
 */
public class DropperCarryAction extends SequentialCommandGroup {
    private static final String LOG_TAG = DropperCarryAction.class.getSimpleName();

    /**
     * Creates a new DropperCarryAction
     *
     * @param dropper    the dropper subsystem
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public DropperCarryAction(DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        addRequirements(dropper, intake);
        addCommands(
                new TransferAction(dropper, intake, robotState),
                new ParallelCommandGroup(
                        new IntakeTuckAfterTransferAction(dropper, intake, robotState),
                        new DropperCarryNoTransferAction(dropper, robotState)
                )
        );
    }
}
