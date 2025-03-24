package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeLoosenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

/**
 * Command to bring the intake to the ready to transfer position after picking up a block. This
 * command is the same as the intake ready to transfer command, but it does not bring the slides
 * in until the intake pitch is safely above the submersible
 */
public class SequentialReadyToTransferAction extends SequentialCommandGroup {
    private static final String LOG_TAG = SequentialReadyToTransferAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new SequentialReadyToTransferAction
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public SequentialReadyToTransferAction(IntakeSubsystem intake, DropperSubsystem dropper,
                                           RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakeClawRotationAction(intake, () -> 60, 0),
                                new WaitUntilCommand(() -> intake.getWristRotation() < 30),
                                new IntakeClawRotationAction(intake, () -> IntakeSubsystem.CLAW_ROTATION_TRANSFER_POSITION, 0)
                        ),
                        new IntakeWristRotationAction(intake,
                                IntakeSubsystem.WRIST_ROTATION_TRANSFER_POSITION, 100),
                        new IntakeWristPitchAction(intake,
                                IntakeSubsystem.WRIST_PITCH_TRANSFER_POSITION, 100),
                        new DropperTransferAction(dropper, robotState)
                ),
                new ParallelCommandGroup(
                        new IntakeLoosenAction(intake, 100),
                        new IntakeSlidesAbsoluteAction(intake,
                                () -> IntakeSubsystem.SLIDES_TRANSFER_POSITION, 0.5)
                ),
                new WaitCommand(100)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setIntakeState(IntakeState.READY_TO_TRANSFER);
        }
    }
}
