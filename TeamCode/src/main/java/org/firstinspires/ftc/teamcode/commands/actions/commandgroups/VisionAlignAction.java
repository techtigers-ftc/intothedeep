package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Command to align the robot to the vision target
 */
public class VisionAlignAction extends ParallelCommandGroup {
    private static final String LOG_TAG = VisionAlignAction.class.getSimpleName();
    private RobotState robotState;

    /**
     * Creates a new VisionAlignAction
     *
     * @param drive      the drive subsystem
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public VisionAlignAction(DriveSubsystem drive, IntakeSubsystem intake, RobotState robotState) {
        addRequirements(intake);
        this.robotState = robotState;
        addCommands(
                new IntakeSlidesAbsoluteAction(intake,
                        () -> intake.getCurrentSlidePositionInches() + robotState.getBlockForwardFine() + 3, 0.5),
                new IntakeClawRotationAction(intake, robotState::getBlockOrientation, 300),
                new TeleHoldPointAction(drive, robotState,
                        () -> robotState.getRobotCurrentPose().getX() +
                                Math.sin(robotState.getRobotCurrentPose().getHeading()) * (robotState.getBlockLateralFine()),
                        () -> robotState.getRobotCurrentPose().getY()
                                - Math.cos(robotState.getRobotCurrentPose().getHeading()) * (robotState.getBlockLateralFine()),
                        () -> robotState.getRobotCurrentPose().getHeading(), 0.3, Math.toRadians(2))
        );
    }

    @Override
    public void initialize() {
        robotState.setVisionAligning(true);
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setVisionAligning(false);
    }
}
