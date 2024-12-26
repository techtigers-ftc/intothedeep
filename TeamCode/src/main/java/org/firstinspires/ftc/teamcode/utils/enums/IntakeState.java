package org.firstinspires.ftc.teamcode.utils.enums;

/**
 * Enum for the five positions of the intake claw
 */
public enum IntakeState {
    /**
     * Position when driving around the field. Claw is tucked in to avoid collisions
     */
    TUCK,

    /**
     * Position of the claw when the slides are being extended; allows the slides to extend
     * without the claw colliding into the submersible or other blocks
     */
    PREPARE_TO_INTAKE,

    /**
     * Slides are extended and the claw is positioned over the block to be picked up, allowing for
     * fine adjustments to be made before picking up the block
     */
    READY_TO_INTAKE,

    /**
     * Claw is closed and facing the robot, but the slides could be extended
     */
    PREPARE_TO_TRANSFER,

    /**
     * Intake is positioned to allow a transfer of the sample to the dropper
     */
    READY_TO_TRANSFER
}