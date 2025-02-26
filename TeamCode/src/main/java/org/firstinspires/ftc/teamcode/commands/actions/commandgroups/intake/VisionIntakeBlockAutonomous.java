package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import java.util.function.DoubleSupplier;

/**
 * Command to move intake to Ready To Transfer.
 */
public class VisionIntakeBlockAutonomous extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;
    private final IntakeSubsystem intake;

    /**
     * Creates a new VisionIntakeBlockAutonomous
     *
     * @param intake               the intake subsystem
     * @param dropper              the dropper subsystem
     * @param clawRotationSupplier the supplier for the claw rotation
     * @param robotState           the robot state
     * @param command              the command to cancel
     */
    public VisionIntakeBlockAutonomous(DriveSubsystem drive,
                                       IntakeSubsystem intake,
                                       DropperSubsystem dropper,
                                       DoubleSupplier clawRotationSupplier,
                                       RobotState robotState, CommandBase command) {
        this.robotState = robotState;
        this.intake = intake;
        addRequirements(intake, dropper);
        addCommands(
                new InstantCommand(() -> robotState.setVisionAligning(true)),
                new ParallelCommandGroup(
                        new IntakeSlidesAbsoluteAction(intake,
                                () -> intake.getCurrentSlidePositionInches() + robotState.getBlockForwardFine() + 3, 0.75),
                        new IntakeClawRotationAction(intake, clawRotationSupplier, 300),
                        new TeleHoldPointAction(drive, robotState,
                                () -> robotState.getRobotCurrentPose().getX() +
                                        Math.sin(robotState.getRobotCurrentPose().getHeading()) * (robotState.getBlockLateralFine()),
                                () -> robotState.getRobotCurrentPose().getY()
                                        - Math.cos(robotState.getRobotCurrentPose().getHeading()) * (robotState.getBlockLateralFine()),
                                () -> robotState.getRobotCurrentPose().getHeading(), 0.5, Math.toRadians(2))
                ),
                new IntakeBlockAutonomous(intake, dropper,
                        robotState, command == null ? this : command)
        );
    }

    /**
     * Overload constructor for if the command doesn't receive a command to cancel
     *
     * @param intake               the intake subsystem
     * @param dropper              the dropper subsystem
     * @param clawRotationSupplier the supplier for the claw rotation
     * @param robotState           the robot state
     */
    public VisionIntakeBlockAutonomous(DriveSubsystem drive,
                                       IntakeSubsystem intake,
                                       DropperSubsystem dropper,
                                       DoubleSupplier clawRotationSupplier,
                                       RobotState robotState) {
        this(drive, intake, dropper, clawRotationSupplier, robotState,null);
    }
}
