package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * Informs the manipulator if the robot failed to pick up a block
 */
public class FailedPickupRumble extends Rumble{
    private IntakeState previousIntakeState;

    /**
     * Constructs a FailedPickupRumble
     * @param manipulatorGamepad the manipulators gamepad
     * @param robotState the state of the robot
     */
    public FailedPickupRumble(GamepadEx manipulatorGamepad, RobotState robotState) {
        super(manipulatorGamepad, robotState);
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
    protected void runRumble() {
        gamepad1.gamepad.rumbleBlips(2);
    }
}
