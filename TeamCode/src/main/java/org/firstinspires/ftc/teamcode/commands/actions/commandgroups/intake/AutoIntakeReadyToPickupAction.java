package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

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
 * Command to move intake to ready to pickup state, tracking the block and its orientation with the small camera
 */
public class AutoIntakeReadyToPickupAction extends SequentialCommandGroup {
    private static final String LOG_TAG = AutoIntakeReadyToPickupAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new AutoIntakeReadyToPickupCommand
     *
     * @param intake                the intake subsystem
     * @param robotState            the robot state
     */
    public AutoIntakeReadyToPickupAction(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeTrackingAction(intake, 50, robotState),
                new IntakeReadyToPickupAction(intake, robotState, robotState::getDetectedFineBlockOrientation)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setIntakeState(IntakeState.READY_TO_PICKUP);
        }
    }
}
