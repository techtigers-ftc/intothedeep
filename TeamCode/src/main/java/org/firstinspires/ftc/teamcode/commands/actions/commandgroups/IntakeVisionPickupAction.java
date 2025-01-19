package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.drive.HoldPointActionTele;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import java.util.function.DoubleSupplier;

/**
 * A command group that automatically picks up a specimen using the vision system and limelight
 */
public class IntakeVisionPickupAction extends SequentialCommandGroup {

    /**
     * Creates a new IntakeVisionPickupAction
     *
     * @param intake          the intake subsystem
     * @param dropper         the dropper subsystem
     * @param drive           the drive subsystem
     * @param robotState      the robot state
     * @param headingSupplier the heading supplier that supplier the heading values to the target position
     */
    public IntakeVisionPickupAction(IntakeSubsystem intake, DropperSubsystem dropper, DriveSubsystem drive, RobotState robotState, DoubleSupplier headingSupplier) {
        addRequirements(intake, dropper, drive);
        addCommands(
                // Aligns the robot to the heading given by the heading supplier
                new HoldPointActionTele(drive, robotState,
                        () -> robotState.getRobotCurrentPose().getX(),
                        () -> robotState.getRobotCurrentPose().getY(),
                        headingSupplier,
                        0.3, Math.toRadians(2)
                ),
                // Moves intake to prepare to pickup and runs intake and drive coarse align
                new ParallelCommandGroup(
                        new IntakePrepareToPickupAction(intake, dropper, robotState, robotState::getBlockForwardCoarse),
                        new HoldPointActionTele(drive, robotState,
                                () -> robotState.getRobotCurrentPose().getX() + Math.sin(robotState.getRobotCurrentPose().getHeading()) * robotState.getBlockLateralCoarse(),
                                () -> robotState.getRobotCurrentPose().getY() - Math.cos(robotState.getRobotCurrentPose().getHeading()) * robotState.getBlockLateralCoarse(),
                                headingSupplier, 0.3, Math.toRadians(2)
                        )
                ),
                // Waits, then runs the fine camera orientation and forward movement alignment
                new WaitCommand(300),
                new IntakeReadyToPickupAction(intake, robotState, () -> intake.getCurrentSlidePositionInches()
                        + robotState.getBlockForwardFine()
                        - VisionSubsystem.INTAKE_CAMERA_OFFSET, robotState::getBlockOrientation)
        );
    }

    /**
     * Overload constructor for intake vision pickup action
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param drive      the drive subsystem
     * @param robotState the robot state
     * @param heading    the heading value
     */
    public IntakeVisionPickupAction(IntakeSubsystem intake, DropperSubsystem dropper, DriveSubsystem drive, RobotState robotState, double heading) {
        this(intake, dropper, drive, robotState, () -> heading);
    }
}
