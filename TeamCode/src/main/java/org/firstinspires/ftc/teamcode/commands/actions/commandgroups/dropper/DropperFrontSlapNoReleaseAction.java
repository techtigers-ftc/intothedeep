package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * A command group that moves the dropper slides in order to hang the
 * specimen on the chamber forwards, without releasing the specimen
 */
public class DropperFrontSlapNoReleaseAction extends SequentialCommandGroup {
    private static final String LOG_TAG = DropperFrontSlapNoReleaseAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new DropperFrontSlapNoReleaseAction
     *
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public DropperFrontSlapNoReleaseAction(DropperSubsystem dropper, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(dropper);
        addCommands(
                new DropperPitchAction(dropper,
                        DropperSubsystem.PITCH_SLAP_POSITION, 0),
                new WaitCommand(300),
                new DropperPitchAction(dropper,
                        DropperSubsystem.PITCH_SLAP_POSITION+20, 0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            robotState.setDropperState(DropperState.FRONT_SLAP);
        }
    }
}
