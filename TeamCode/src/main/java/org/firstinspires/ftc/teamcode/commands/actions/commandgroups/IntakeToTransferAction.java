package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakePitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
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
                new IntakeCloseAction(intake),
                new ParallelCommandGroup(
                        new IntakePitchAction(intake, 180, 300),
                        new IntakeRotationAction(intake, 90, 300)
                ),
//                new ParallelCommandGroup(
                        new IntakePitchAction(intake, 0, 300),
                        new IntakeSlidesAbsoluteAction(intake, 0, 0.25)
//                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setIntakeState(IntakeState.TRANSFER);
        robotState.setBlockPosition(RobotBlockPosition.INTAKE);

    }
}
