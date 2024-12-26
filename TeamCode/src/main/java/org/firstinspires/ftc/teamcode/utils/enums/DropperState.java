package org.firstinspires.ftc.teamcode.utils.enums;

/**
 * The state of the dropper transfer or dropping
 */
public enum DropperState {
    /**
     * The dropper is in the pre-transfer state
     * This State is slightly higher than the transfer state
     */
    PRE_TRANSFER,
    /**
     * The dropper is in the transfer state
     */
    TRANSFER,
    /**
     * The Dropper is Ready to Drop a Specimen When the Robot is Slapping Forward
     */
    FORWARD_CARRY,
    /**
     * The Dropper is Ready to Drop a Specimen When the Robot is Slapping Backward
     */
    BACKWARD_CARRY,
    /**
     * The dropper is ready to drop to the high basket
     */
    HIGH_BASKET,
    /**
     * The dropper is ready to drop to the low basket
     */
    LOW_BASKET
}
