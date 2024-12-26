package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * Command to move intake to ready to intake state
 */
public class IntakeReadyToPickupAction extends ParallelCommandGroup {
    private static final String LOG_TAG = IntakeReadyToPickupAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new IntakeReadyToIntakeCommand
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeReadyToPickupAction(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeWristRotationAction(intake,
                        IntakeSubsystem.WRIST_ROTATION_READY_TO_PICKUP_POSITION, 500),
                new IntakeClawRotationAction(intake,
                        IntakeSubsystem.CLAW_ROTATION_READY_TO_PICKUP_POSITION, 500),
                new IntakeWristPitchAction(intake,
                        IntakeSubsystem.WRIST_PITCH_READY_TO_PICKUP_POSITION, 500)
        );
    }

    @Override
    public void initialize() {
        if (robotState.getIntakeState() != IntakeState.PREPARE_TO_PICKUP) {
            RobotLog.ww(LOG_TAG, "Invalid intake position: %s", robotState.getIntakeState());
            robotState.setError(RobotError.INVALID_INTAKE_POSITION);
            this.cancel();
        } else {
            RobotLog.dd(LOG_TAG, "Executing IntakeReadyToPickupAction command from state: %s", robotState.getIntakeState());
            super.initialize();
        }
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        if (!interrupted) {
            robotState.setIntakeState(IntakeState.READY_TO_PICKUP);
        }
    }
}
