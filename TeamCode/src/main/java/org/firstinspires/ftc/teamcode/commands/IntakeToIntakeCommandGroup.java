package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeOpenActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakePitchActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeRotationActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeSlidesAbsoluteActionCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

/**
 * A command group that moves the intake system to the intake position
 */
public class IntakeToIntakeCommandGroup extends SequentialCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new IntakeToIntakeCommandGroup
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeToIntakeCommandGroup(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeOpenActionCommand(intake),
                new ParallelCommandGroup(
                        new IntakeSlidesAbsoluteActionCommand(intake, 20, 0.5),
                        new IntakeRotationActionCommand(intake, 90, 300)
                ),
                new IntakePitchActionCommand(intake, 180, 300)
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setIntakeState(IntakeState.READY_TO_GRAB);
    }
}
