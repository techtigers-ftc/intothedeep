package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
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
public class DropperTransferAction extends SequentialCommandGroup {
    private RobotState robotState;
    private static final String LOG_TAG = DropperTransferAction.class.getSimpleName();

    /**
     * Creates a new DropperTransferAction.
     *
     * @param dropper The dropper subsystem.
     * @param intake The intake subsystem.
     * @param robotState The robot state.
     */
    public DropperTransferAction(DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper, intake);
        addCommands(
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_TRANSFER_POSITION, 500),
                new DropperCloseAction(dropper, 200),
                new IntakeOpenAction(intake, 200),
                new IntakeWristPitchAction(intake, 70, 500)
        );
    }

    @Override
    public void initialize() {
        RobotLog.dd(LOG_TAG, "Executing command from state: %s", robotState.getIntakeState());
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
