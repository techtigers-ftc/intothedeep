package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

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

    @Override
    protected void runRumble() {
        gamepad1.gamepad.runRumbleEffect(new com.qualcomm.robotcore.hardware.Gamepad.RumbleEffect.Builder()
                .addStep(0.7, 1, 75)
                .addStep(0, 0, 150)
                .addStep(0.7, 1, 75)
                .build()
        );
    }
}
