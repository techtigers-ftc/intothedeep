package team.techtigers.base;

import com.arcrobotics.ftclib.command.SubsystemBase;

/**
 * Extension of SubsystemBase that adds an init() and close() method. This
 * method is called when the robot is disabled.
 */
public class CloseableSubsystem extends SubsystemBase {
    protected String tag;

    /**
     * Constructor for CloseableSubsystem
     */
    public CloseableSubsystem() {
        tag = this.getClass().getSimpleName();
    }

    /**
     * Optionally sets the tag for the subsystem. The tag defaults to the
     * class name.
     *
     * @param tag The tag to set
     */
    protected void setTag(String tag) {
        this.tag = tag;
    }

    /**
     * Optional method to close any hardware resources that should be closed when the robot is
     * disabled. This can be overridden by child classes.
     */
    public void close() {
    }

    /**
     * Optional method to initialize the subsystem prior to the OpMode starting. This method can be
     * overridden by child classes.
     */
    public void init() {
    }
}