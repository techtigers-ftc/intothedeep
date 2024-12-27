package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that moves the dropper to the forward high chamber drop position, with the
 * specimen upside down, ready to be clipped downwards onto the high chamber.
 * The NT stands for "No Transfer"
 */
public class DropperForwardCarryNoTransferAction extends ParallelCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new DropperForwardCarryNTAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperForwardCarryNoTransferAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperSlidesAbsoluteAction(dropper, 0, 0.5),
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_CHAMBER_POSITION, 300),
                new DropperRotationAction(dropper,
                        DropperSubsystem.ROTATION_FRONT_SLAP_POSITION, 300)
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            robotState.setDropperState(DropperState.FORWARD_CARRY);
        }
    }
}
