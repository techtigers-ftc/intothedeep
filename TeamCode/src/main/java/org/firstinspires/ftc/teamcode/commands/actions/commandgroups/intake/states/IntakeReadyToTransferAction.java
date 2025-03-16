package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command that moves the dropper to the pre-transfer position if it is tucked into the robot
 */
public class IntakeReadyToTransferAction extends CommandBase {
    private final SequentialCommandGroup parallelReadyToTransfer;
    private final SequentialCommandGroup sequentialReadyToTransfer;
    private CommandBase currentCommand;
    private final IntakeSubsystem intakeSubsystem;
    private final double INTAKE_SAFETY_LIMIT = 7;

    /**
     * Constructor for IntakeReadyToTransferAction
     *
     * @param intake     the dropper subsystem
     * @param robotState the robot state
     */
    public IntakeReadyToTransferAction(IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        this.intakeSubsystem = intake;
        this.parallelReadyToTransfer = new ParallelIntakeReadyToTransferAction(intake, dropper, robotState);
        this.sequentialReadyToTransfer = new SequentialIntakeReadyToTransferAction(intake, dropper, robotState);
    }

    @Override
    public void initialize() {
        currentCommand = intakeSubsystem.getCurrentSlidePositionInches() > INTAKE_SAFETY_LIMIT ? parallelReadyToTransfer : sequentialReadyToTransfer;
        currentCommand.initialize();
    }

    @Override
    public void execute() {
        currentCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return currentCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        currentCommand.end(interrupted);
    }
}
