package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that moves the dropper slides in order to hang the specimen on the chamber forwards
 */
public class DropperFrontSlapAction extends SequentialCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new DropperFrontSlapAction
     *
     * @param dropper the dropper subsystem
     * @param robotState the robot state
     */
    public DropperFrontSlapAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_FRONT_SLAP_POSITION, 0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted){
            robotState.setDropperState(DropperState.FRONT_SLAP);
        }
    }
}
