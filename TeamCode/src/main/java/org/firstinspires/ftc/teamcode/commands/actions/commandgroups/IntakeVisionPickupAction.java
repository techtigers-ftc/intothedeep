package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.drive.DriveCoarseAlignAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
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
    public IntakeVisionPickupAction(IntakeSubsystem intake, DropperSubsystem dropper, DriveSubsystem drive, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake, dropper, drive);
        addCommands(
                // MOVES INTAKE TO PREPARE FOR PICKUP AND RUNS COARSE ALIGNMENT
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new IntakeSlidesAbsoluteAction(intake, robotState::getBlockForwardCoarse, 1),
                                        new IntakeWristRotationAction(intake,
                                                IntakeSubsystem.WRIST_ROTATION_PREPARE_TO_PICKUP_POSITION, 200),
                                        new IntakeClawRotationAction(intake,
                                                () -> IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION, 200),
                                        new IntakeWristPitchAction(intake,
                                                IntakeSubsystem.WRIST_PITCH_PREPARE_TO_PICKUP_POSITION, 200)
                                ),
                                new ParallelCommandGroup(
                                        new DropperPitchAction(dropper, DropperSubsystem.PITCH_PRE_TRANSFER_POSITION, 50),
                                        new IntakeOpenAction(intake)
                                )),
                        new DriveCoarseAlignAction(drive, robotState, 0.1)
                ),
                new IntakeSlidesAbsoluteAction(intake, () -> intake.getCurrentSlidePositionInches()
                        + robotState.getBlockForwardFine()
                        - VisionSubsystem.INTAKE_CAMERA_OFFSET, 0.5),
                new ParallelCommandGroup(
                        new IntakeWristRotationAction(intake,
                                IntakeSubsystem.WRIST_ROTATION_READY_TO_PICKUP_POSITION, 100),
                        new IntakeClawRotationAction(intake,
                                robotState::getBlockOrientation, 100),
                        new IntakeWristPitchAction(intake,
                                IntakeSubsystem.WRIST_PITCH_READY_TO_PICKUP_POSITION, 100)
                ),
                new IntakeOpenAction(intake, 0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setIntakeState(IntakeState.READY_TO_PICKUP);
        robotState.setBlockPosition(RobotBlockPosition.INTAKE);
    }
}
