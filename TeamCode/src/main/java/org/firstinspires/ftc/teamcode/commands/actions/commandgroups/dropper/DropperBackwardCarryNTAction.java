package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that moves the dropper to the backward high chamber drop position, with the
 * specimen upside down, ready to be clipped upwards onto the high chamber.
 * The NT stands for "No Transfer"
 */
public class DropperBackwardCarryNTAction extends ParallelCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new DropperBackwardCarryNTAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperBackwardCarryNTAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_CHAMBER_POSITION, 300),
                new DropperRotationAction(dropper,
                        DropperSubsystem.ROTATION_BACK_SLAP_POSITION, 300)
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            robotState.setDropperState(DropperState.BACKWARD_CARRY);
        }
    }
}
