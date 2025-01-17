package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeLoosenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * Command to move intake to ready to transfer state.
 */
public class IntakeReadyToTransferAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakeReadyToTransferAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new IntakeReadyToTransferAction
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeReadyToTransferAction(IntakeSubsystem intake,
                                       RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeSlidesAbsoluteAction(intake, () -> 0.5, 0.5)
        );
    }

    @Override
    public void initialize() {
        if (robotState.getIntakeState() != IntakeState.PREPARE_TO_TRANSFER) {
            RobotLog.ww(LOG_TAG, "Invalid intake position: %s", robotState.getIntakeState());
            robotState.setError(RobotError.INVALID_INTAKE_POSITION);
            this.cancel();
        } else {
            RobotLog.dd(LOG_TAG, "Executing command from state: %s", robotState.getIntakeState());
            robotState.clearError(RobotError.INVALID_INTAKE_POSITION);
            super.initialize();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setIntakeState(IntakeState.READY_TO_TRANSFER);
        }
    }
}
