package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that moves the dropper to the transfer position
 */
public class DropperPreTransferAction extends ParallelCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new DropperToTransferAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperPreTransferAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_PRE_TRANSFER_POSITION,
                        300),
                new DropperRotationAction(dropper, DropperSubsystem.ROTATION_TRANSFER_POSITION, 300),
                new DropperSlidesAbsoluteAction(dropper, 0, 0.5),
                new DropperOpenAction(dropper)
                );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted){
            robotState.setDropperState(DropperState.PRE_TRANSFER);
            robotState.setCurrentGear(DriveGears.NOT_ENGAGED);
        }
    }
}
