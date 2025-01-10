package team.techtigers.base;

import android.os.Environment;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

import team.techtigers.core.utils.GlobalState;


/**
 * A CommandOpMode from FTCLib that allows for custom telemetry and more
 * features
 */
public abstract class BaseOpMode extends CommandOpMode {
    protected GlobalState robotState;
    private CloseableSubsystem[] subsystems;
    private boolean serialization = true;
    private ArrayList<GlobalState> robotStates = new ArrayList<>();

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
     * Disables the process of copying robotState and setting it to a file
     */
    protected void disableSerialization() {
        serialization = false;
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
                update();
                telemetry.update();
                if (serialization) {
                    robotStates.add(robotState);
                    RobotLog.dd("Serialization", "Saved new robot state");
                }
            }
        } finally {
            if (serialization) {
                String filename = "states.json";
                String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+"cache";
                File directory = new File(directoryPath);
                //noinspection ResultOfMethodCallIgnored
                directory.mkdir();
                RobotLog.dd("Serialization", "Preparing to write to file %s", filename);
                try (ObjectOutputStream oos = new ObjectOutputStream(new FileOutputStream(directoryPath+"/"+filename))) {
                    for (GlobalState state : robotStates) {
                        state.write(oos);
                    }
                    RobotLog.dd("Serialization", "Writing to file %s", filename);
                } catch (IOException e) {
                    RobotLog.ee("Serialization", "error: %s", e);
                }
            }

            reset();
            // Cleaning up after execution, whether or not there are no errors
            for (CloseableSubsystem subsystem : subsystems) {
                subsystem.close();
            }
            end();
        }
    }
}