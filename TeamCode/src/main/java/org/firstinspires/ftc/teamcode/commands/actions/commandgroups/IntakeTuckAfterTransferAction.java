package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * A command group that tucks the intake after doing a transfer
 */
public class IntakeTuckAfterTransferAction extends SequentialCommandGroup {
    private static final String LOG_TAG = TransferAction.class.getSimpleName();

    /**
     * Creates a new IntakeTuckAfterTransferAction
     *
     * @param dropper    The dropper subsystem.
     * @param intake     The intake subsystem.
     * @param robotState The robot state.
     */
    public IntakeTuckAfterTransferAction(DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        addRequirements(intake);
        addCommands(
                new WaitUntilCommand(() -> dropper.getCurrentSlidePositionInches() > 2),
                new IntakeTuckAction(intake, robotState)
        );
    }
}
