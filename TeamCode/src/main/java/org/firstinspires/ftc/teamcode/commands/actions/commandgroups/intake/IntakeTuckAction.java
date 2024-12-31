package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * Command to tuck the intake in.
 */
public class IntakeTuckAction extends ParallelCommandGroup {
    private final RobotState robotState;
    private static final String LOG_TAG = IntakeTuckAction.class.getSimpleName();

    /**
     * Creates a new IntakeTuckCommand
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeTuckAction(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_TUCK_POSITION, 200),
                new IntakeSlidesAbsoluteAction(intake, 0, 0.5),
                new IntakeCloseAction(intake),
                new IntakeClawRotationAction(intake, IntakeSubsystem.CLAW_ROTATION_TUCK_POSITION, 200),
                new IntakeWristRotationAction(intake, IntakeSubsystem.WRIST_ROTATION_TUCK_POSITION, 200)
        );
    }

    @Override
    public void initialize() {
        if (robotState.getIntakeState() != IntakeState.PREPARE_TO_PICKUP
                && robotState.getIntakeState() != IntakeState.READY_TO_TRANSFER) {
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
            robotState.setIntakeState(IntakeState.TUCK);
            robotState.setCurrentGear(DriveGears.NOT_ENGAGED);
        }
    }
}
