package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TargetIntakeRobotPositionCalculator;

import team.techtigers.core.paths.Waypoint;

/**
 * An action to use the limelight to roughly align the robot to a region of samples.
 * This action will move the robot to the region of samples and position the intake over the blocks.
 */
public class IntakeCoarseAlignAction extends ParallelCommandGroup {
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
        addRequirements(intake);

        addCommands(
                new TeleHoldPointAction(drive, robotState,
                        () -> targetPositions[0],
                        () -> targetPositions[1],
                        () -> targetPositions[2],
                        0.5, Math.toRadians(2)
                ),
                new IntakeReadyToPickupAction(intake, robotState,
                        () -> targetPositions[3] - LimelightSubsystem.SLIDES_OFFSET - 6)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        robotState.setVisionAligning(true);
        robotState.setCoarseCameraMode(true);

        Waypoint blockPos = robotState.getAbsoluteBlockPosition().getAbsoluteBlockPosition();
        targetPositions = TargetIntakeRobotPositionCalculator.getTargetIntakePositionFine(
                robotState.getRobotCurrentPose(),
                blockPos
        );

        double distanceFromTargetPos = Math.hypot(
                targetPositions[0] - robotState.getRobotCurrentPose().getX(),
                targetPositions[1] - robotState.getRobotCurrentPose().getY()
        );

        RobotLog.dd("Auto Debug", "Distance from Target (in): %f", distanceFromTargetPos);
        RobotLog.dd("Auto Debug", "Robot Current Position X: %f Y: %f", robotState.getRobotCurrentPose().getX(), robotState.getRobotCurrentPose().getY());
        RobotLog.dd("Auto Debug", "Target Position X: %f Y: %f", targetPositions[0], targetPositions[1]);
        RobotLog.dd("Auto Debug", "Block Absolute Position X: %f Y: %f", blockPos.getX(), blockPos.getY());
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robotState.setVisionAligning(false);
        robotState.setCoarseCameraMode(false);
    }
}
