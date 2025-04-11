package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command group to automatically do the level 3 ascent
 */
public class LevelThreeAscentCommandGroup extends SequentialCommandGroup {

    /**
     * Constructs a new LevelThreeAscentCommandGroup
     * @param robotState the state of the robot
     * @param ascent the ascent subsystem
     * @param dropper the dropper subsystem
     * @param drive the drive subsystem
     */
    public LevelThreeAscentCommandGroup(RobotState robotState,
                                        AscentSubsystem ascent,
                                        DropperSubsystem dropper,
                                        DriveSubsystem drive) {

        addRequirements(ascent, dropper, drive);
        addCommands(
                new InstantCommand(ascent::engageAscent),
                new StartAscentCommandGroup(robotState, ascent, dropper),
                new WaitCommand(2000),
                new MoveAscentSlidesCommand(robotState, ascent, dropper,
                        drive, -0.75),
                new MoveAscentSlidesCommand(robotState, ascent, dropper,
                        drive, 22),
                new WaitUntilCommand(() -> Math.abs(robotState.getRobotPitch()) < 7),
                new MoveAscentSlidesCommand(robotState, ascent, dropper,
                        drive, -1)
        );
    }
}
