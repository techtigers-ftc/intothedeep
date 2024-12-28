package org.firstinspires.ftc.teamcode.utils;

public class SlidingAverageCalculator {
    private static double[] values;
    private static double average;
    private static int index;
    private static double sum;
    private int capacity;
    private static int count;

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
