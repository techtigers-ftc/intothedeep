package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that moves the dropper to the off the wall intake position to pick up specimens
 */
public class DropperToWallAction extends ParallelCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new DropperToWallAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperToWallAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperOpenAction(dropper),
                new DropperSlidesAbsoluteAction(dropper, 0, 0.5),
                new DropperPitchAction(dropper, 135, 300),
                new DropperRotationAction(dropper, 180, 300)
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setDropperState(DropperState.WALL_INTAKE);
    }
}
