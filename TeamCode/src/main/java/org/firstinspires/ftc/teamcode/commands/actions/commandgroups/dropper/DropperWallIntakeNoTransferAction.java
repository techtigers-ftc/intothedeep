package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotError;

/**
 * A command group that gets the robot ready to do a wall intake without transferring a sample
 */
public class DropperWallIntakeNoTransferAction extends ParallelCommandGroup {
    private static final String LOG_TAG = DropperWallIntakeNoTransferAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new DropperWallIntakeAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperWallIntakeNoTransferAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperSlidesAbsoluteAction(dropper, DropperSubsystem.SLIDES_WALL_INTAKE_POSITION, 0.25),
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_WALL_INTAKE_POSITION, 200),
                new DropperRotationAction(dropper,
                        DropperSubsystem.ROTATION_WALL_INTAKE_POSITION, 200),
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
