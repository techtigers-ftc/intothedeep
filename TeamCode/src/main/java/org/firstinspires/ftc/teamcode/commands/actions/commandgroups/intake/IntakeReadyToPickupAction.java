package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

import java.util.function.DoubleSupplier;

/**
 * Command to move intake to ready to intake state
 */
public class IntakeReadyToPickupAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakeReadyToPickupAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new IntakeReadyToIntakeCommand
     *
     * @param intake                the intake subsystem
     * @param robotState            the robot state
     * @param slidePositionSupplier the supplier for the target slide position
     * @param clawRotationSupplier  the supplier for the target claw rotation
     */
    public IntakeReadyToPickupAction(IntakeSubsystem intake, RobotState robotState,
                                     DoubleSupplier slidePositionSupplier, DoubleSupplier clawRotationSupplier) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeSlidesAbsoluteAction(intake, slidePositionSupplier, 0.5),
                new ParallelCommandGroup(
                        new IntakeWristRotationAction(intake,
                                IntakeSubsystem.WRIST_ROTATION_READY_TO_PICKUP_POSITION, 200),
                        new IntakeClawRotationAction(intake,
                                clawRotationSupplier, 200),
                        new IntakeWristPitchAction(intake,
                                IntakeSubsystem.WRIST_PITCH_READY_TO_PICKUP_POSITION, 200)
                ),
                new IntakeOpenAction(intake, 0)
        );
    }

    /**
     * Overloaded constructor that takes a target claw rotation position instead of a supplier,
     * Also defaults to no slide movement.
     *
     * @param intake               the intake subsystem
     * @param robotState           the robot state
     * @param clawRotationPosition the target claw rotation position
     */
    public IntakeReadyToPickupAction(IntakeSubsystem intake, RobotState robotState,
                                     double clawRotationPosition) {
        this(intake, robotState, intake::getCurrentSlidePositionInches, () -> clawRotationPosition);
    }

    @Override
    public void initialize() {
        if (robotState.getIntakeState() != IntakeState.PREPARE_TO_PICKUP) {
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
            robotState.setIntakeState(IntakeState.READY_TO_PICKUP);
        }
    }
}
