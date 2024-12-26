package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;

/**
 * A command group that moves the intake system to the pickup position, ready to pick up a sample
 * or specimen
 */
public class IntakeToPickupAction extends ParallelCommandGroup {
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
                new IntakeSlidesAbsoluteAction(intake, 10, 0.5),
                new IntakeWristRotationAction(intake,
                        IntakeSubsystem.WRIST_ROTATION_READY_TO_INTAKE_POSITION, 500),
                new IntakeClawRotationAction(intake,
                        IntakeSubsystem.CLAW_ROTATION_READY_TO_INTAKE_POSITION, 500),
                new IntakeWristPitchAction(intake,
                        IntakeSubsystem.WRIST_PITCH_PICKUP_POSITION, 500),
                new IntakeOpenAction(intake)
        );
    }

    @Override
    public void end(boolean interrupted) {
        // TODO: Fix this
//        robotState.setIntakeState(IntakeState.READY_TO_PICKUP);
        robotState.setCurrentGear(DriveGears.ENGAGED);
    }
}
