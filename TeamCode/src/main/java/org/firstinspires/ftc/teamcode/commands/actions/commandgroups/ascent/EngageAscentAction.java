package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.ascent;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.ascent.AscentEngageAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

public class EngageAscentAction extends SequentialCommandGroup {
    /**
     * Constructs a new EngageAscentAction
     * @param robotState the state of the robot
     * @param ascent the ascent subsystem, used to engage the ascent
     * @param dropper the dropper subsystem, used to move the slides
     */
    public EngageAscentAction(RobotState robotState, AscentSubsystem ascent, DropperSubsystem dropper) {
        addCommands(
                new DropperSlidesAbsoluteAction(dropper,
                        AscentSubsystem.ASCENT_INITIAL_HEIGHT+1, 0.5),
                new AscentEngageAction(ascent, 0)
        );
    }
}
