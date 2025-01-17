package team.techtigers.core.utils;

import androidx.annotation.NonNull;

//import com.google.gson.Gson;

import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import team.techtigers.core.paths.Waypoint;

/**
 * A class to store information that is global to the entire robot. This is
 * intended to be extended from, and child classes can add in additional
 * attributes that are desired in the state.
 */
public class GlobalState implements Serializable, Cloneable {
    private long startTime;

    /**
     * Initializes a new GlobalState with a timer
     */
    public GlobalState() {
        resetTimer();
    }

    /**
     * @return the amount of time elapsed since the last reset
     */
    public long getRunTime() {
        return System.currentTimeMillis() - startTime;
    }

    /**
     * Resets the timer running in GlobalState
     */
    public void resetTimer() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public GlobalState clone() throws CloneNotSupportedException {
        GlobalState clone = (GlobalState) super.clone();
        clone.startTime = this.startTime;

        return clone;
    }

    protected int extractValues(HashMap<Integer, String> values) throws IllegalAccessException {
        values.put(0, String.valueOf(this.startTime));
        values.put(1, String.valueOf(System.currentTimeMillis()));
        return 2;
    }
    protected int extractValues(HashMap<Integer, String> values, GlobalState previousState) throws IllegalAccessException {
        values.put(1, String.valueOf(System.currentTimeMillis()));
        return 2;
    }

    protected String convertToString(Object obj) {
        if(obj.getClass() == Waypoint.class) {
            return String.format("{x: %f, y: %f, heading: %f}", ((Waypoint) obj).getX(), ((Waypoint) obj).getY(), ((Waypoint) obj).getHeading());
        } else {
            return obj.toString();
        }
    }

    public String toJson() throws IllegalAccessException {
        HashMap<Integer, String> values = new HashMap<>();
        extractValues(values);
        //Make json like string out of dictionary
        StringBuilder jsonString = new StringBuilder("{");
        for (Map.Entry<Integer, String> entry : values.entrySet()) {
            jsonString.append("\"").append(entry.getKey()).append(":\"")
                    .append(entry.getValue()).append("\",");
        }
        // Remove the trailing comma and close the JSON string
        if (jsonString.length() > 1) {
            jsonString.setLength(jsonString.length() - 2);
        }
        jsonString.append("}");

        return jsonString.toString();
    }

}
