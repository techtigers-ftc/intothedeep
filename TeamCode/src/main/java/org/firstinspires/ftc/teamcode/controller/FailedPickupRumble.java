package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * Informs the manipulator if the robot failed to pick up a block
 */
public class FailedPickupRumble implements ControllerEffect {
    private IntakeState previousIntakeState;
    private final RobotState robotState;
    private final GamepadEx gamepad;
    private final Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 100)
            .addStep(0, 0, 100)
            .addStep(1, 1, 100)
            .build();
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
    public void updateEffect() {
        boolean pickedUpBlock = previousIntakeState == IntakeState.READY_TO_PICKUP &&
                robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER;
        boolean noBlockInIntake = robotState.getBlockPosition() != RobotBlockPosition.INTAKE;
        if(pickedUpBlock && noBlockInIntake) {
            gamepad.gamepad.runRumbleEffect(rumbleEffect);
        }

        previousIntakeState = robotState.getIntakeState();
    }
}
