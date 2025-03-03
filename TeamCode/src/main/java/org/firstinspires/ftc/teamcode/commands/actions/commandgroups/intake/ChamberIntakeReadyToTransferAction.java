package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

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
 * Command to move intake to ready to transfer state from the chamber
 */
public class ChamberIntakeReadyToTransferAction extends SequentialCommandGroup {
    private static final String LOG_TAG = ChamberIntakeReadyToTransferAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new ChamberIntakeReadyToTransferAction
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public ChamberIntakeReadyToTransferAction(IntakeSubsystem intake, DropperSubsystem dropper,
                                              RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakeClawRotationAction(intake, () -> 30, 0),
                                new WaitUntilCommand(() -> intake.getWristRotation() < 30),
                                new IntakeClawRotationAction(intake, () -> IntakeSubsystem.CLAW_ROTATION_TRANSFER_POSITION, 100)
                        ),
                        new IntakeWristRotationAction(intake,
                                IntakeSubsystem.WRIST_ROTATION_TRANSFER_POSITION, 200),
                        new IntakeWristPitchAction(intake,
                                IntakeSubsystem.WRIST_PITCH_TRANSFER_POSITION - 20, 200),
                        new SequentialCommandGroup(
                                new WaitCommand(150),
                                new DropperTransferAction(dropper, robotState)
                        )
                ),
                new ParallelCommandGroup(
                        new IntakeWristPitchAction(intake,
                                IntakeSubsystem.WRIST_PITCH_TRANSFER_POSITION, 100),
                        new IntakeLoosenAction(intake, 200),
                        new IntakeSlidesAbsoluteAction(intake,
                                () -> IntakeSubsystem.SLIDES_TRANSFER_POSITION, 1)
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
