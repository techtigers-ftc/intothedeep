package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

/**
 * An action to use the limelight to roughly align the robot to a region of samples.
 * This action will move the robot to the region of samples and position the intake over the blocks.
 */
public class IntakeCoarseAlignAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakeCoarseAlignAction.class.getSimpleName();
    private RobotState robotState;

    /**
     * Creates a new IntakeCoarseAlignAction
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public IntakeCoarseAlignAction(DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake, dropper);

        addCommands(
                new IntakePrepareToPickupAction(intake, dropper, robotState),
                new WaitUntilCommand(() -> robotState.getCoarseBlockDetectionState() == BlockDetectionState.DETECTED),
                new ParallelCommandGroup(
                        new TeleHoldPointAction(drive, robotState,
                                () -> robotState.getRobotCurrentPose().getX() +
                                        Math.sin(robotState.getRobotCurrentPose().getHeading()) * (robotState.getBlockLateralCoarse()),
                                () -> robotState.getRobotCurrentPose().getY()
                                        - Math.cos(robotState.getRobotCurrentPose().getHeading()) * (robotState.getBlockLateralCoarse()),
                                () -> robotState.getRobotCurrentPose().getHeading(), 0.5, Math.toRadians(2)
                        ),
                        new IntakeReadyToPickupAction(intake, robotState,
                                () -> intake.getCurrentSlidePositionInches() + robotState.getBlockForwardCoarse() - 3,
                                () -> IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION
                        )
                )
        );
    }

    @Override
    public void initialize() {
        robotState.setVisionAligning(true);
        robotState.setCoarseCameraMode(true);
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        if (!interrupted) {
            robotState.setVisionAligning(false);
        }
    }
}
