package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command that moves the dropper to the pre-transfer position if it is tucked into the robot
 */
public class DropperSafetyPreTransfer extends CommandBase {
    private final ParallelCommandGroup prepareToTransfer;
    private final InstantCommand noPrepareToTransfer;
    private final DropperSubsystem dropperSubsystem;
    private final double DROPPER_SAFETY_LIMIT = 3;
    private CommandBase currentCommand;

    /**
     * Constructor for DropperSafetyPreTransfer
     *
     * @param dropperSubsystem the dropper subsystem
     * @param robotState       the robot state
     */
    public DropperSafetyPreTransfer(DropperSubsystem dropperSubsystem, RobotState robotState) {
        this.dropperSubsystem = dropperSubsystem;
        this.prepareToTransfer = new DropperPreTransferAction(dropperSubsystem, robotState);
        this.noPrepareToTransfer = new InstantCommand();
    }

    @Override
    public void initialize() {
        currentCommand = dropperSubsystem.getCurrentSlidePositionInches() > DROPPER_SAFETY_LIMIT ? noPrepareToTransfer : prepareToTransfer;
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
