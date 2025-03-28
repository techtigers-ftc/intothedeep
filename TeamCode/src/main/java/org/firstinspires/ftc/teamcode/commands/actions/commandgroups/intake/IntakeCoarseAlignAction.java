package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TargetRobotPoseCalculator;

import team.techtigers.core.paths.Waypoint;

/**
 * An action to use the limelight to roughly align the robot to a region of samples.
 * This action will move the robot to the region of samples and position the intake over the blocks.
 */
public class IntakeCoarseAlignAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakeCoarseAlignAction.class.getSimpleName();
    private final RobotState robotState;
    private double[] targetPositions;

    /**
     * Creates a new IntakeCoarseAlignAction
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeCoarseAlignAction(DriveSubsystem drive, IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        targetPositions = new double[5];
        addRequirements(intake);

        addCommands(
                new InstantCommand(() -> robotState.setVisionAligning(true)),
                new ParallelCommandGroup(
                        new TeleHoldPointAction(drive, robotState,
                                () -> targetPositions[0],
                                () -> targetPositions[1],
                                () -> targetPositions[2],
                                0.5, Math.toRadians(2)
                        ),
                        new IntakeReadyToPickupAction(intake, robotState,
                                () -> targetPositions[3] - LimelightSubsystem.SLIDES_OFFSET - LimelightSubsystem.LIMELIGHT_FINE_OFFSET - 1)
                )
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        robotState.setCoarseCameraMode(true);
        if (!robotState.isBlockDetected()) {
            // TODO: Replace with something that won't crash the robot
            RobotLog.ww(LOG_TAG, "Skipping fine align because block is not detected");
            throw new IllegalStateException("Block not detected");
        }

        Waypoint blockPos = robotState.getAbsoluteBlockPosition();
        targetPositions = TargetRobotPoseCalculator.getTargetIntakePosition(
                robotState.getRobotCurrentPose(),
                blockPos
        );
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robotState.setVisionAligning(false);
        robotState.setCoarseCameraMode(false);
    }
}
