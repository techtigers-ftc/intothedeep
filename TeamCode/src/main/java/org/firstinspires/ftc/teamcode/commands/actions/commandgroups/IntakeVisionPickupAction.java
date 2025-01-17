package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.drive.HoldPointAction;
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
                // Aligns the robot to a heading of 0
                new HoldPointAction(drive, robotState,
                        () -> robotState.getRobotCurrentPose().getX(),
                        () -> robotState.getRobotCurrentPose().getY(),
                        () -> 0,
                        0.3, Math.toRadians(2)
                ),
                // Moves intake to prepare to pickup and runs intake and drive coarse align
                new ParallelCommandGroup(
                        new IntakePrepareToPickupAction(intake, dropper, robotState, robotState::getBlockForwardCoarse),
                        new HoldPointAction(drive, robotState,
                                () -> robotState.getRobotCurrentPose().getX(),
                                () -> robotState.getRobotCurrentPose().getY() - robotState.getBlockLateralCoarse(),
                                () -> 0, 0.3, Math.toRadians(2)
                                )
                ),
                // Waits, then runs the fine camera alignment
                new WaitCommand(300),
//                new HoldPointAction(drive, robotState,
//                        () -> robotState.getRobotCurrentPose().getX(),
//                        () -> robotState.getRobotCurrentPose().getY() - robotState.getBlockLateralFine(),
//                        () -> 0, 0.3, Math.toRadians(2)
//                ),
                new IntakeReadyToPickupAction(intake, robotState, () -> intake.getCurrentSlidePositionInches()
                        + robotState.getBlockForwardFine()
                        - VisionSubsystem.INTAKE_CAMERA_OFFSET, robotState::getBlockOrientation)
        );
    }
}
