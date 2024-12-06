package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperCloseActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperPitchActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperRotationActionCommand;
import org.firstinspires.ftc.teamcode.commands.actions.dropper.DropperSlidesActionCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that transfers the sample from the intake to the dropper and
 * moves the dropper system to the high chamber drop position, with the specimen
 * upside down, ready to be clipped upwards onto the high chamber
 */
public class DropperToHighChamberFromWallDropCommandGroup extends SequentialCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new DropperToHighChamberFromWallDropCommandGroup
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperToHighChamberFromWallDropCommandGroup(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperCloseActionCommand(dropper, 100),
                new ParallelCommandGroup(
                        new DropperSlidesActionCommand(dropper, 25, 0.5),
                        new DropperPitchActionCommand(dropper, 300, 300),
                        new DropperRotationActionCommand(dropper, 180, 300)
                        // Drop forward: 1
                        // Pick up from intake: 0.6
                        //Guessing under 70 degress for pitch to be able to clip onto the high
                        // chamber upside down
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setDropperState(DropperState.DROP);
    }
}
