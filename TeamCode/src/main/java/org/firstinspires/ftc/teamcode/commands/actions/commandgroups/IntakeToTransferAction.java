package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * A command group that moves the intake system to the transfer position
 */
public class IntakeToTransferAction extends SequentialCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new IntakeToTransferAction
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeToTransferAction(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeWristPitchAction(intake,
                        IntakeSubsystem.WRIST_PITCH_PECK_POSITION, 200),
                new IntakeCloseAction(intake, 150),
                new ParallelCommandGroup(
                        new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_TRANSFER_POSITION, 400),
                        new IntakeWristRotationAction(intake, IntakeSubsystem.WRIST_ROTATION_TRANSFER_POSITION, 200),
                        new IntakeClawRotationAction(intake, IntakeSubsystem.CLAW_ROTATION_TRANSFER_POSITION, 400),
                        new IntakeSlidesAbsoluteAction(intake, 0, 0.25)
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setIntakeState(IntakeState.TRANSFER);
        robotState.setBlockPosition(RobotBlockPosition.INTAKE);
        robotState.setCurrentGear(DriveGears.NOT_ENGAGED);

    }
}
