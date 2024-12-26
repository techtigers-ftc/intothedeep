package org.firstinspires.ftc.teamcode.utils.enums;

/**
 * Enum for the five positions of the intake claw
 */
public enum IntakeClawPosition {
    // Position when driving around
    TUCK,
    // Position when searching for a sample to intake; doesn't collide w/ submersible
    PREPARE_TO_INTAKE,
    // Position right before picking up a block to ensure you are getting the right block
    // Claw will collide with the submersible
    READY_TO_INTAKE,
    // Claw is in the position to transfer but the slides have not retracted
    PREPARE_TO_TRANSFER,
    // Claw is in the position to transfer, awaiting the dropper to pick up the sample
    READY_TO_TRANSFER
}