package org.firstinspires.ftc.teamcode.utils;


import team.techtigers.base.visualdisplay.Color;

public enum GearSelection {
    /**
     * The first gear (slowest)
     */
    SLOW(0, Color.RED),

    /**
     * The second gear (fastest)
     */
    FAST(1,Color.GREEN);

    public final int value;
    public final Color color;

    GearSelection(int value, Color color) {
        this.value = value;
        this.color = color;
    }

    /**
     * Gets the next gear
     * @return the next gear
     */
    public GearSelection getNext() {
        if (this.value == 0) {
            return GearSelection.FAST;
        } else {
            return GearSelection.SLOW;
        }
    }
}
