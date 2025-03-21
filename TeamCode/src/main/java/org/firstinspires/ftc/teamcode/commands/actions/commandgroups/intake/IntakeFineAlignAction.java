package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TargetIntakeRobotPositionCalculator;

import java.util.function.DoubleSupplier;

import team.techtigers.core.paths.Waypoint;

/**
 * Command to align to a block using fine camera vision
 */
public class IntakeFineAlignAction extends ParallelCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;
    private double[] targetPositions;

    /**
     * Creates a new IntakeFineAlignAction
     *
     * @param drive                the drive subsystem
     * @param intake               the intake subsystem
     * @param clawRotationSupplier the supplier for the claw rotation
     * @param robotState           the robot state
     */
    public IntakeFineAlignAction(DriveSubsystem drive, IntakeSubsystem intake,
                                 DoubleSupplier clawRotationSupplier,
                                 RobotState robotState) {
        this.robotState = robotState;
        targetPositions = new double[4];
        addRequirements(intake);
        addCommands(
                new IntakeSlidesAbsoluteAction(intake,
                        () -> targetPositions[3] - LimelightSubsystem.SLIDES_OFFSET - 3, 0.75, 0.3),
                new IntakeClawRotationAction(intake, clawRotationSupplier, 150),
                new TeleHoldPointAction(drive, robotState,
                        () -> targetPositions[0],
                        () -> targetPositions[1],
                        () -> targetPositions[2],
                        0.5, Math.toRadians(2))
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        robotState.setVisionAligning(true);
        if (!robotState.getAbsoluteBlockPosition().isBlockDetected()) {
            RobotLog.ww(LOG_TAG, "Skipping fine align because block is not detected");
            throw new IllegalStateException("Block not detected");
            // TODO: Possibly cancel command so robot doesn't crash during a match
//            this.cancel();
        }

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
    }
}
