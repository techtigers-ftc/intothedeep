package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.Arrays;

/**
 * A class to calculate the sliding average used to find how many amps the robot is using
 */
public class SlidingAverageCalculator {
    private double[] values;
    private double average;
    private int index;
    private double sum;
    private int capacity;
    private int count;

    /**
     * Creates a new SlidingAverageCalculator with the given capacity.
     * @param capacity the total sample size before resting old values
     */
    public SlidingAverageCalculator(int capacity) {
        this.capacity = capacity;
        values = new double[capacity];
        this.clear();
    }

    /**
     * Adds a value to the calculator.
     * @param value the value to add in amps
     */
    public void add(double value) {
        sum -= values[index];
        values[index] = value;
        sum += value;
        count = Math.min(count + 1, capacity);
        index = (index + 1) % capacity;
        average = sum / count;
    }

    /**
     * @return the average of the values in the calculator
     */
    public double getAverage() {
        return average;
    }

    /**
     * Clears the values in the calculator.
     */
    public void clear() {
        for (int index = 0; index < capacity; index++) {
            values[index] = 0;
        }
        average = 0;
        index = 0;
        sum = 0;
        count = 0;
    }
}
