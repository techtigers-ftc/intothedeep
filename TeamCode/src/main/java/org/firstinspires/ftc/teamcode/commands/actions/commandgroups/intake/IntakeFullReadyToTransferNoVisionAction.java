package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.ReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeCheckSensorAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * Command to pick up a sample once the robot is hovered over a block, and bring the intake to the
 * transfer position. The block is ready to be picked up by the dropper.
 */
public class IntakeFullReadyToTransferNoVisionAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;
    private final IntakeSubsystem intake;

    /**
     * Creates a new IntakeFullReadyToTransferNoVisionAction
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     * @param command    the command to cancel
     */
    public IntakeFullReadyToTransferNoVisionAction(IntakeSubsystem intake,
                                                   DropperSubsystem dropper,
                                                   RobotState robotState, CommandBase command) {
        this.robotState = robotState;
        this.intake = intake;
        addRequirements(intake, dropper);
        addCommands(
                new IntakePrepareToTransferAction(intake, robotState),
                new IntakeCheckSensorAction(robotState, command == null ? this : command),
                new InstantCommand(() -> robotState.setVisionAligning(false)),
                new ReadyToTransferAction(intake, dropper, robotState)
        );
    }

    /**
     * Overload constructor for if the command doesn't receive a command to cancel
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public IntakeFullReadyToTransferNoVisionAction(IntakeSubsystem intake,
                                                   DropperSubsystem dropper,
                                                   RobotState robotState) {
        this(intake, dropper, robotState, null);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robotState.setVisionAligning(false);
        if (!interrupted || robotState.getDropperState() == DropperState.TRANSFER) {
            robotState.setIntakeState(IntakeState.READY_TO_TRANSFER);
            robotState.setBlockPosition(RobotBlockPosition.INTAKE);
            robotState.setCurrentGear(DriveGears.NOT_ENGAGED);
        } else {
            robotState.setIntakeState(IntakeState.READY_TO_PICKUP);
            robotState.setCurrentGear(DriveGears.ENGAGED);
            intake.setWristPitchAbsolute(IntakeSubsystem.WRIST_PITCH_READY_TO_PICKUP_POSITION);
            intake.setWristRotationAbsolute(IntakeSubsystem.WRIST_ROTATION_READY_TO_PICKUP_POSITION);
            intake.openClaw();
            intake.setClawRotationAbsolute(IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION);
            if (!robotState.isManualIntakeSelected()) {
                intake.moveSlidesRelative(-3);
            }
        }
    }
}
