package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import java.util.function.DoubleSupplier;

/**
 * Command to align to a block using fine camera vision
 */
public class IntakeFineAlignAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;
    private final IntakeSubsystem intake;

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
        this.intake = intake;
        addRequirements(intake);
        addCommands(
                new InstantCommand(() -> robotState.setVisionAligning(true)),
                new ParallelCommandGroup(
                        new IntakeSlidesAbsoluteAction(intake,
                                () -> intake.getCurrentSlidePositionInches() + robotState.getBlockForwardFine() + 3, 0.75, 0.3),
                        new IntakeClawRotationAction(intake, clawRotationSupplier, 150),
                        new TeleHoldPointAction(drive, robotState,
                                () -> robotState.getRobotCurrentPose().getX() +
                                        Math.sin(robotState.getRobotCurrentPose().getHeading()) * (robotState.getBlockLateralFine()),
                                () -> robotState.getRobotCurrentPose().getY()
                                        - Math.cos(robotState.getRobotCurrentPose().getHeading()) * (robotState.getBlockLateralFine()),
                                () -> robotState.getRobotCurrentPose().getHeading(), 0.5, Math.toRadians(2))
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robotState.setVisionAligning(false);
    }
}
