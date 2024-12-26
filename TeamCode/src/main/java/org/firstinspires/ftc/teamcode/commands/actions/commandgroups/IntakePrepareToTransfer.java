package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * Command to move intake to prepare Prepare To Transfer.
 */
public class IntakePrepareToTransfer extends SequentialCommandGroup {
    private final RobotState robotState;
    private static final String LOG_TAG = IntakePrepareToIntakeAction.class.getSimpleName();

    /**
     * Creates a new IntakeTuckCommand
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakePrepareToTransfer(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeWristPitchAction(intake,
                        IntakeSubsystem.WRIST_PITCH_PECK_POSITION, 200),
                new IntakeCloseAction(intake, 150),
                new ParallelCommandGroup(
                        new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_TRANSFER_POSITION, 400),
                        new IntakeWristRotationAction(intake, IntakeSubsystem.WRIST_ROTATION_TRANSFER_POSITION, 200),
                        new IntakeClawRotationAction(intake, IntakeSubsystem.CLAW_ROTATION_TRANSFER_POSITION, 400)
                )
        );
    }

    @Override
    public void initialize() {
        if (robotState.getIntakeState() != IntakeState.READY_TO_INTAKE) {
            RobotLog.ww(LOG_TAG, "Invalid intake position: %s", robotState.getIntakeState());
            robotState.setError(RobotError.INVALID_INTAKE_POSITION);
            this.cancel();
        } else {
            RobotLog.dd(LOG_TAG, "Executing Intake Prepare To Transfer command from state: %s", robotState.getIntakeState());
            super.initialize();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setIntakeState(IntakeState.PREPARE_TO_TRANSFER);
        }
    }
}
