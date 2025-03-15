package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import java.util.function.DoubleSupplier;

/**
 * Command to move the intake to ready to pickup state, with the claw open
 */
public class IntakeReadyToPickupAction extends ParallelCommandGroup {
    private static final String LOG_TAG = IntakeReadyToPickupAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new IntakeReadyToPickupAction
     *
     * @param intake                the intake subsystem
     * @param robotState            the robot state
     * @param slidePositionSupplier the supplier for the target slide position
     */
    public IntakeReadyToPickupAction(IntakeSubsystem intake, RobotState robotState, DoubleSupplier slidePositionSupplier) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeSlidesAbsoluteAction(intake, slidePositionSupplier, 0.5),
                new IntakeWristRotationAction(intake, IntakeSubsystem.WRIST_ROTATION_READY_TO_PICKUP_POSITION, 300),
                new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_READY_TO_PICKUP_POSITION, 300),
                new IntakeClawRotationAction(intake, () -> IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION, 100),
                new IntakeOpenAction(intake, 0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setCoarseCameraMode(false);
            robotState.setIntakeState(IntakeState.READY_TO_PICKUP);
        }
    }
}
