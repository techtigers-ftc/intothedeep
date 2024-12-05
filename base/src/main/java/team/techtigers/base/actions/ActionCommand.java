package team.techtigers.base.actions;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

/**
 * Command to handle the running of an action command, which is a synchronized
 * movement of servos and/or motors
 */
public class ActionCommand extends CommandBase {
    /**
     * Initializes a new ActionCommand
     *
     * @param subsystems     The subsystems that are required
     */
    public ActionCommand(Subsystem... subsystems) {
        addRequirements(subsystems);
    }

    public ActionCommand() {
        throw new IllegalArgumentException("ActionCommand requires " +
                "at least 1 subsystem to be passed in");
    }
}
