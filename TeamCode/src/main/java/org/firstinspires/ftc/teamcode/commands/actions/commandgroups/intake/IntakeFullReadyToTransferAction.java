package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

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
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * Command to move intake to Ready To Transfer.
 */
public class IntakeFullReadyToTransferAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;
    private final IntakeSubsystem intake;
    private double lastClawRotation;

    /**
     * Creates a new IntakeFullReadyToTransferAction
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public IntakeFullReadyToTransferAction(IntakeSubsystem intake,
                                           DropperSubsystem dropper,
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
                        new IntakeClawRotationAction(intake, () -> IntakeSubsystem.CLAW_ROTATION_TRANSFER_POSITION, 100),
                        new IntakeWristRotationAction(intake,
                                IntakeSubsystem.WRIST_ROTATION_TRANSFER_POSITION, 300)
                ),
                new IntakeCheckSensorAction(robotState, this),
                new ParallelCommandGroup(
                        new DropperPitchAction(dropper,
                                DropperSubsystem.PITCH_TRANSFER_POSITION, 100),
                        new IntakeLoosenAction(intake, 350)
                ),
                new IntakeCloseAction(intake, 50),
                new IntakeSlidesAbsoluteAction(intake, () -> IntakeSubsystem.SLIDES_TRANSFER_POSITION, 1)
        );
    }

    @Override
    public void initialize() {
        if (robotState.getIntakeState() != IntakeState.READY_TO_PICKUP) {
            RobotLog.ww(LOG_TAG, "Invalid intake position: %s", robotState.getIntakeState());
            robotState.setError(RobotError.INVALID_INTAKE_POSITION);
            this.cancel();
        } else {
            RobotLog.dd(LOG_TAG, "Executing command from state: %s", robotState.getIntakeState());
            robotState.clearError(RobotError.INVALID_INTAKE_POSITION);
            super.initialize();
        }
        lastClawRotation = intake.getClawRotation();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setIntakeState(IntakeState.READY_TO_TRANSFER);
            robotState.setBlockPosition(RobotBlockPosition.INTAKE);
            robotState.setCurrentGear(DriveGears.NOT_ENGAGED);
        } else {
            intake.setClawRotationAbsolute(lastClawRotation);
            intake.setWristPitchAbsolute(IntakeSubsystem.WRIST_PITCH_READY_TO_PICKUP_POSITION);
            intake.setWristRotationAbsolute(IntakeSubsystem.WRIST_ROTATION_READY_TO_PICKUP_POSITION);
            intake.openClaw();
        }
    }
}
