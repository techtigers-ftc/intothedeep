package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeTuckAfterTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.TransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper system to the high basket drop position
 */
public class DropperHighBasketAction extends SequentialCommandGroup {
    private static final String LOG_TAG = DropperHighBasketAction.class.getSimpleName();

    /**
     * Creates a new DropperHighBasketAction
     *
     * @param dropper    the dropper subsystem
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public DropperHighBasketAction(DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        addRequirements(dropper, intake);
        addCommands(
                new TransferAction(dropper, intake, robotState),
                new ParallelCommandGroup(
                        new IntakeTuckAfterTransferAction(dropper, intake, robotState),
                        new DropperHighBasketNoTransferAction(dropper, robotState)
                )
        );
    }
}
