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
 * moves the dropper system to the low basket drop position
 */
public class DropperLowBasketAction extends SequentialCommandGroup {
    private static final String LOG_TAG = DropperLowBasketAction.class.getSimpleName();

    /**
     * Creates a new DropperLowBasketAction
     *
     * @param dropper    the dropper subsystem
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public DropperLowBasketAction(DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        addRequirements(dropper, intake);
        addCommands(
                new TransferAction(dropper, intake, robotState),
                new ParallelCommandGroup(
                        new IntakeTuckAfterTransferAction(dropper, intake, robotState),
                        new DropperLowBasketNoTransferAction(dropper, robotState)
                )
        );
    }
}
