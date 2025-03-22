package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * Informs the manipulator if the robot failed to pick up a block
 */
public class FailedPickupRumble implements Rumble{
    private IntakeState previousIntakeState;
    private final RobotState robotState;
    private final GamepadEx gamepad;

    /**
     * Constructs a FailedPickupRumble
     * @param gamepad the gamepad to rumble
     * @param robotState the state of the robot
     */
    public FailedPickupRumble(GamepadEx gamepad, RobotState robotState) {
        this.gamepad = gamepad;
        this.robotState = robotState;
        previousIntakeState = robotState.getIntakeState();
    }

    /**
     * Rumbles in two short blips if the robot failed an intake
     */
    @Override
    public void updateRumble() {
        boolean pickedUpBlock = previousIntakeState == IntakeState.READY_TO_PICKUP &&
                robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER;
        boolean noBlockInIntake = robotState.getBlockPosition() != RobotBlockPosition.INTAKE;
        if(pickedUpBlock && noBlockInIntake) {
            runRumble();
        }

        previousIntakeState = robotState.getIntakeState();
    }

    /**
     * Runs two short blips of rumble
     */
    @Override
    public void runRumble() {
        gamepad.gamepad.rumbleBlips(2);
    }
}
