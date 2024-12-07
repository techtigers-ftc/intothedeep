package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperCloseActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperPitchActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperSlidesAbsoluteActionCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that grabs the specimen from the wall and
 * moves the dropper to the high chamber drop position, with the specimen
 * upside down, ready to be clipped upwards onto the high chamber
 */
public class DropperHighChamberWallAction extends SequentialCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new DropperHighChamberWallAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperHighChamberWallAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperCloseActionCommand(dropper, 100),
                new DropperSlidesAbsoluteActionCommand(dropper, 5, 0.5),
                new ParallelCommandGroup(
                        new DropperSlidesAbsoluteActionCommand(dropper, 25, 0.5),
                        new DropperPitchActionCommand(dropper, 300, 300)
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setDropperState(DropperState.DROP);
    }
}
