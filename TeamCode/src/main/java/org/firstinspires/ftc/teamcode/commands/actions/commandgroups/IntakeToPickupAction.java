package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakePitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

/**
 * A command group that moves the intake system to the pickup position, ready to pick up a sample
 * or specimen
 */
public class IntakeToPickupAction extends SequentialCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new IntakeToPickupAction
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeToPickupAction(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeOpenAction(intake),
                new ParallelCommandGroup(
                        new IntakeSlidesAbsoluteAction(intake, 10, 0.5),
                        new IntakeRotationAction(intake, 90, 300)
                ),
                new IntakePitchAction(intake, 180, 300)
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setIntakeState(IntakeState.READY_TO_PICKUP);
    }
}
