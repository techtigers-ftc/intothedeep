package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Vector;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

/**
 * A rumble that tells the manipulator which side of the block is relative to the robot
 **/
public class VisionAllignmentRumble extends Rumble {
    private BlockDetectionState blockDetectionState;
    private IntakeState currentIntakeState;
    private GamepadEx manipulatorGamepad;

    /**
     * Constructor for VisionAllignmentRumble class
     *
     * @param manipulatorGamepad Gamepad for manipulator
     * @param robotState         The robot state to use
     **/
    public VisionAllignmentRumble(GamepadEx manipulatorGamepad, RobotState robotState) {
        super(manipulatorGamepad, robotState);
    }


    // Check if the block is detected by the robot for vision alignment
    @Override
    public void updateRumble() {
        boolean intaking = robotState.getIntakeState() == IntakeState.PREPARE_TO_PICKUP
                || robotState.getIntakeState() == IntakeState.READY_TO_PICKUP;
        if (blockDetectionState == BlockDetectionState.DETECTED && intaking) {
            runRumble();
        }
    }

    @Override
    public void runRumble() {
        Vector robotI = new Vector(1, robotState.getRobotCurrentPose().getHeading());
        Vector robotJ = new Vector(1, robotState.getRobotCurrentPose().getHeading() + Math.PI / 2);
        double lateral = robotState.getAbsoluteBlockCoordinates().getX() * robotI.getXComponent()
                + robotState.getAbsoluteBlockCoordinates().getY() * robotJ.getXComponent();
        if(lateral < 0) {
            manipulatorGamepad.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(0, calculateRumbleIntensity(lateral), Gamepad.RUMBLE_DURATION_CONTINUOUS)
                    .build());
        } else {
            manipulatorGamepad.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(calculateRumbleIntensity(lateral), 0, Gamepad.RUMBLE_DURATION_CONTINUOUS)
                    .build());
        }
    }
    private double calculateRumbleIntensity(double lateral) {
        return 1/(1 + Math.exp(-1.5 * Math.abs(lateral)));
    }
}


