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
 */
public class DropperUndersideCarryNoTransferAction extends ParallelCommandGroup {
    private static final String LOG_TAG = DropperUndersideCarryNoTransferAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new DropperCarryNoTransferAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperUndersideCarryNoTransferAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperSlidesAbsoluteAction(dropper, 5, 0.5),
                new DropperPitchAction(dropper, 80, 200),
                new DropperRotationAction(dropper,
                        DropperSubsystem.ROTATION_TRANSFER_POSITION, 100)
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            robotState.setDropperState(DropperState.FORWARD_CARRY);
        }
    }
}
