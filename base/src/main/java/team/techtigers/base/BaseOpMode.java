package team.techtigers.base;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Subsystem;


/**
 * A CommandOpMode from FTCLib that allows for custom telemetry and more
 * features
 */
public abstract class BaseOpMode extends CommandOpMode {
    private CloseableSubsystem[] subsystems;
    private boolean enableUpdate = true;

    /**
     * Disables the running of the update method. This is useful if you want to
     * stop running telemetry in an opmode
     */
    protected void disableUpdate() {
        enableUpdate = false;
    }

    /**
     * Enable the running of the update method. This is useful if you want to
     * continue running telemetry in an opmode after stopping it
     */
    protected void enableUpdate() {
        enableUpdate = true;
    }

    /**
     * Method run during the loop. Needed methods and telemetry should be placed here.
     */
    protected void update() {
    }

    /**
     * Method that is called just after the OpMode starts. It is recommended to
     * use subsystems init() method if possible, but this is also an option.
     */
    protected void justAfterStart() {
    }

    /**
     * Method that is called as the OpMode ends. It is recommended to
     * use subsystems end() method if possible, but this is also an option.
     */
    protected void end() {
    }

    /**
     * Child classes must invoke this method to ensure that the subsystems are properly registered
     * and cleaned up after the OpMode is finished.
     *
     * @param subsystems The subsystems to register
     */
    protected void registerSubsystems(CloseableSubsystem... subsystems) {
        this.subsystems = subsystems;
        super.register(subsystems);
    }

    @Override
    public void register(Subsystem... subsystems) {
        throw new UnsupportedOperationException("Use registerSubsystems() instead of register()");
    }

    @Override
    public void runOpMode() {
        subsystems = new CloseableSubsystem[0];

        try {
            initialize();
            waitForStart();
            for (CloseableSubsystem subsystem : subsystems) {
                subsystem.init();
            }
            justAfterStart();

            // run the scheduler
            while (!isStopRequested() && opModeIsActive()) {
                run();
                if (enableUpdate) {
                    update();
                    telemetry.update();
                }
            }
        } finally {
            reset();
            // Cleaning up after execution, whether or not there are no errors
            for (CloseableSubsystem subsystem : subsystems) {
                subsystem.close();
            }
            end();
        }
    }
}