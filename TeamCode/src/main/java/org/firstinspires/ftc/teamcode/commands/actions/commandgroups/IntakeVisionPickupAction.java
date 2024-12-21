package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.drive.DriveCoarseAlignAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeCoarseAlignAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * A command group that automatically picks up a specimen using the vision system and limelight
 */
public class IntakeVisionPickupAction extends SequentialCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new IntakeVisionPickupAction
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeVisionPickupAction(IntakeSubsystem intake, DriveSubsystem drive, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake, drive);
        addCommands(
                // MOVE SERVOS TO PICKUP AND RUNS COARSE ALIGNMENT
                new ParallelCommandGroup(
                        new IntakeWristRotationAction(intake, IntakeSubsystem.WRIST_ROTATION_PICKUP_POSITION, 350),
                        new IntakeClawRotationAction(intake, IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION, 200),
                        new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_PICKUP_POSITION, 200),
                        new IntakeOpenAction(intake),
                        new DriveCoarseAlignAction(drive, robotState, 0.25),
                        new IntakeCoarseAlignAction(intake, robotState, 0.5, 5)
                ),
                new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_PECK_POSITION, 250),
                new IntakeCloseAction(intake, 150),
                // MOVE INTAKE TO PICKUP POSITION
                new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_TRANSFER_POSITION, 125),
                new ParallelCommandGroup(
                        new IntakeWristRotationAction(intake, IntakeSubsystem.WRIST_ROTATION_TRANSFER_POSITION, 200),
                        new IntakeClawRotationAction(intake, IntakeSubsystem.CLAW_ROTATION_TRANSFER_POSITION, 300),
                        new IntakeSlidesAbsoluteAction(intake, 0, 0.25)
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setIntakeState(IntakeState.TRANSFER);
        robotState.setBlockPosition(RobotBlockPosition.INTAKE);
    }
}
