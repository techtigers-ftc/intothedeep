package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.drive.DriveCoarseAlignAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
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
        addRequirements(intake, drive);
        addCommands(
                // MOVES INTAKE TO PREPARE FOR PICKUP AND RUNS COARSE ALIGNMENT
                new ParallelCommandGroup(
                        new IntakePrepareToPickupAction(intake, dropper, robotState, robotState::getBlockForwardCoarse),
                        new DriveCoarseAlignAction(drive, robotState, 0.1)
                ),
                new IntakeReadyToPickupAction(intake, robotState,
                        () -> intake.getCurrentSlidePositionInches()
                            + robotState.getBlockForwardFine()
                            - VisionSubsystem.INTAKE_CAMERA_OFFSET,
                        robotState::getBlockOrientation),
                new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_PECK_POSITION, 350),
                new IntakeCloseAction(intake, 250),
                // MOVE INTAKE TO PICKUP POSITION
                new IntakePrepareToTransferAction(intake, dropper, robotState)
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setIntakeState(IntakeState.READY_TO_TRANSFER);
        robotState.setBlockPosition(RobotBlockPosition.INTAKE);
    }
}
