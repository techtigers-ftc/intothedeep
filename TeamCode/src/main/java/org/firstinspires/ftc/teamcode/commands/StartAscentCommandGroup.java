package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.ascent.AscentEngageAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command group to engage the ascent and the jacks, while bringing the
 * slides up
 */
public class StartAscentCommandGroup extends SequentialCommandGroup {
    /**
     * Constructs a new StartAscentCommandGroup
     *
     * @param robotState the state of the robot
     * @param ascent     the ascent subsystem, used to engage the ascent
     * @param dropper    the dropper subsystem, used to move the slides
     */
    public StartAscentCommandGroup(RobotState robotState, AscentSubsystem ascent, DropperSubsystem dropper) {
        addRequirements(ascent, dropper);
        addCommands(
                new DropperSlidesAbsoluteAction(dropper,
                        AscentSubsystem.ASCENT_SLIDES_INITIAL_HEIGHT + 1, 0.5),
                new InstantCommand(dropper::stopSlides),
                new AscentEngageAction(ascent, robotState, 1000)
        );
    }
}
