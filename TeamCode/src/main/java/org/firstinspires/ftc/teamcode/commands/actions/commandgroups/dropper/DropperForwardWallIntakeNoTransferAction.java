package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that gets the robot ready to do a wall intake without transferring a sample
 */
public class DropperForwardWallIntakeNoTransferAction extends ParallelCommandGroup {
    private static final String LOG_TAG = DropperForwardWallIntakeNoTransferAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new DropperForwardWallIntakeNoTransferAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperForwardWallIntakeNoTransferAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperSlidesAbsoluteAction(dropper, DropperSubsystem.SLIDES_WALL_INTAKE_POSITION, 0.25),
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_FRONT_WALL_INTAKE_POSITION, 200),
                new DropperRotationAction(dropper,
                        DropperSubsystem.ROTATION_TRANSFER_POSITION, 200),
                new DropperOpenAction(dropper, 100)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setDropperState(DropperState.WALL_INTAKE);
        }
    }
}
