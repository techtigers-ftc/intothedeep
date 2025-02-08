package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeCheckSensorAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeLoosenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import java.util.function.DoubleSupplier;

/**
 * Command to move intake to Prepare To Transfer.
 */
public class IntakePrepareToTransferAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;
    private final IntakeSubsystem intake;
    private double lastClawRotation;

    /**
     * Creates a new IntakePrepareToTransferAction
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public IntakePrepareToTransferAction(IntakeSubsystem intake,
                                         DropperSubsystem dropper, DoubleSupplier targetSlidePosition,
                                         RobotState robotState) {
        this.robotState = robotState;
        this.intake = intake;
        lastClawRotation = 90;
        addRequirements(intake, dropper);
        addCommands(
                new IntakeWristPitchAction(intake,
                        IntakeSubsystem.WRIST_PITCH_PECK_POSITION, 100),
                new IntakeCloseAction(intake, 150),
                new ParallelCommandGroup(
                        new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_TRANSFER_POSITION, 100),
                        new IntakeClawRotationAction(intake, () -> IntakeSubsystem.CLAW_ROTATION_TRANSFER_POSITION, 100)
                ),
                new IntakeCheckSensorAction(robotState, this),
                new ParallelCommandGroup(
                        new IntakeSlidesAbsoluteAction(intake, targetSlidePosition, 1),
                        new IntakeWristRotationAction(intake,
                                IntakeSubsystem.WRIST_ROTATION_TRANSFER_POSITION, 100),
                        new DropperPitchAction(dropper,
                                DropperSubsystem.PITCH_TRANSFER_POSITION, 100)
                ),
                new IntakeLoosenAction(intake, 300),
                new IntakeCloseAction(intake, 50)
        );
    }

    /**
     * Overloaded constructor which sets the slide position to 5 automatically
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public IntakePrepareToTransferAction(IntakeSubsystem intake,
                                         DropperSubsystem dropper,
                                         RobotState robotState) {
        this(intake, dropper, () -> 5, robotState);
    }

    @Override
    public void initialize() {
        lastClawRotation = intake.getClawRotation();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setIntakeState(IntakeState.PREPARE_TO_TRANSFER);
            robotState.setBlockPosition(RobotBlockPosition.INTAKE);
            robotState.setCurrentGear(DriveGears.NOT_ENGAGED);
        } else {
            intake.setClawRotationAbsolute(lastClawRotation);
            intake.setWristPitchAbsolute(IntakeSubsystem.WRIST_PITCH_READY_TO_PICKUP_POSITION);
            intake.openClaw();
        }
    }
}
