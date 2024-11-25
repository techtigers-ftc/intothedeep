package team.techtigers.core.utils;

import java.util.HashMap;

/**
 * Saves values from one opmode to be used in another
 */
public class RobotSaveState {
    private final HashMap<String, Object> state;
    private static RobotSaveState instance;

    private RobotSaveState() {
        state = new HashMap<>();
    }

    /**
     * If there is an instance of the class, returns it
     * Otherwise, creates a new instance and returns it
     *
     * @return The instance of the class
     */
    public static RobotSaveState getInstance() {
        if (instance == null) {
            instance = new RobotSaveState();
        }

        return instance;
    }

    /**
     * Gets a value from the current robot state
     *
     * @param key The key to be used for identification
     * @return The current robot state
     */
    public Object getState(String key) {
        if (!state.containsKey(key)) {
            throw new IllegalStateException("No state has been set for key: " + key);
        } else {
            return state.get(key);
        }
    }

    /**
     * Sets the current robot state, requires a string for identification
     * and object for the value to be saved
     *
     * @param key   The key to be used for identification
     * @param value The value to be saved
     */
    public void setState(String key, Object value) {
        if (key == null) {
            throw new NullPointerException("Key cannot be null");
        } else {
            state.put(key, value);
        }
    }

    /**
     * Resets the instance of this class to be null
     */
    public static void reset() {
        instance = null;
    }
}