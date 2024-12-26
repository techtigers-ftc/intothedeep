package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * Command to move intake to prepare for intake state
 */
public class IntakeReadyToIntakeAction extends ParallelCommandGroup {
    private static final String LOG_TAG = IntakeReadyToIntakeAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new IntakeToPrepareCommand
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeReadyToIntakeAction(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeWristRotationAction(intake,
                        IntakeSubsystem.WRIST_ROTATION_READY_TO_INTAKE_POSITION, 500),
                new IntakeClawRotationAction(intake,
                        IntakeSubsystem.CLAW_ROTATION_READY_TO_INTAKE_POSITION, 500),
                new IntakeWristPitchAction(intake,
                        IntakeSubsystem.WRIST_PITCH_READY_TO_INTAKE_POSITION, 500)
        );
    }

    @Override
    public void initialize() {
        if (robotState.getIntakeState() != IntakeState.PREPARE_TO_INTAKE) {
            RobotLog.ww(LOG_TAG, "Invalid intake position: %s", robotState.getIntakeState());
            robotState.setError(RobotError.INVALID_INTAKE_POSITION);
            this.cancel();
        } else {
            RobotLog.dd(LOG_TAG, "Executing IntakeReadyToIntakeAction command from state: %s", robotState.getIntakeState());
            super.initialize();
        }
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        if (!interrupted) {
            robotState.setIntakeState(IntakeState.READY_TO_INTAKE);
        }
    }
}
