package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeCloseActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakePitchActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeRotationActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeSlidesAbsoluteActionCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

/**
 * A command group that moves the intake system to the transfer position
 */
public class IntakeToTransferCommandGroup extends SequentialCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new IntakeToTransferCommandGroup
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeToTransferCommandGroup(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeCloseActionCommand(intake),
                new ParallelCommandGroup(
                        new IntakePitchActionCommand(intake, 180, 300),
                        new IntakeRotationActionCommand(intake, 90, 300)
                ),
                new ParallelCommandGroup(
                        new IntakePitchActionCommand(intake, 0, 300),
                        new IntakeSlidesAbsoluteActionCommand(intake, 1, 0.5)
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setIntakeState(IntakeState.TRANSFER);
    }
}
