package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

/**
 * Command to pick up a block
 */
public class IntakePrepareToTransferAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new IntakePrepareToTransfer
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakePrepareToTransferAction(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_PECK_POSITION, 50),
                new IntakeCloseAction(intake, 175),
                new ParallelCommandGroup(
//                        new IntakeClawRotationAction(intake, () -> IntakeSubsystem.CLAW_ROTATION_TRANSFER_POSITION, 100),
                        new IntakeWristPitchAction(intake,
                                IntakeSubsystem.WRIST_PITCH_TRANSFER_POSITION - 10, 125)
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robotState.setVisionAligning(false);
        robotState.setIntakeState(IntakeState.PREPARE_TO_TRANSFER);
        robotState.setCurrentGear(DriveGears.NOT_ENGAGED);
    }
}
