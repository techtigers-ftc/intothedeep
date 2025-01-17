package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.drive.DriveCoarseAlignAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

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
                        new IntakePrepareToPickupAction(intake, dropper, robotState, robotState::getBlockForwardCoarse),
                        new DriveCoarseAlignAction(drive, robotState, 0.1)
                ),
                new IntakeReadyToPickupAction(intake, robotState, () -> intake.getCurrentSlidePositionInches()
                        + robotState.getBlockForwardFine()
                        - VisionSubsystem.INTAKE_CAMERA_OFFSET, robotState::getBlockOrientation)
        );
    }
}
