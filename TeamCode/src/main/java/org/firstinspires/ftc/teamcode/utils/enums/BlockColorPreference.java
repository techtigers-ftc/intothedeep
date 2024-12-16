package org.firstinspires.ftc.teamcode.utils.enums;

/**
 * Enum for the color of the block that should be looked for
 */
public enum BlockColorPreference {
    ALLIANCE(0),
    YELLOW(1),
    ANY(2);

    public final int value;
    BlockColorPreference(int value) {
        this.value = value;
    }

    /**
     * Gets the next block color preference
     * @return the next block color preference
     */
    public BlockColorPreference getNext() {
        if (this.value == 0) {
            return BlockColorPreference.YELLOW;
        } else if (this.value == 1) {
            return BlockColorPreference.ANY;
        } else {
            return BlockColorPreference.ALLIANCE;
        }
    }
}
