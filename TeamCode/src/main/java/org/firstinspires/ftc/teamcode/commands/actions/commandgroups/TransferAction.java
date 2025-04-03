package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * A command group that transfers a block from the intake to the dropper.
 */
public class TransferAction extends SequentialCommandGroup {
    private static final String LOG_TAG = TransferAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new TransferAction.
     *
     * @param dropper    The dropper subsystem.
     * @param intake     The intake subsystem.
     * @param robotState The robot state.
     */
    public TransferAction(DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper, intake);
        addCommands(
                new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_TRANSFER_POSITION - 10, 25),
                new DropperCloseAction(dropper, 50),
                new IntakeOpenAction(intake, 75)
        );
    }

    @Override
    public void initialize() {
        robotState.clearError(RobotError.INVALID_DROPPER_POSITION);
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            robotState.setBlockPosition(RobotBlockPosition.DROPPER);
        }
    }
}
