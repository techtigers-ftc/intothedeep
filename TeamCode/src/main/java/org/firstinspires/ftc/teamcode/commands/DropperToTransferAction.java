package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperOpenActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperPitchActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperRotationActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperSlidesAbsoluteActionCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that moves the dropper to the transfer position
 */
public class DropperToTransferAction extends ParallelCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new DropperToTransferAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperToTransferAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperOpenActionCommand(dropper),
                new DropperSlidesAbsoluteActionCommand(dropper, 0, 0.5),
                new DropperPitchActionCommand(dropper, 210, 300),
                new DropperRotationActionCommand(dropper, 0, 300)
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setDropperState(DropperState.TRANSFER);
    }
}
