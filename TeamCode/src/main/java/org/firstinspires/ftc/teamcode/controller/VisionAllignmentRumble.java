package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

/**
 A rumble that tells the manipulator which side of the block is relative to the robot
 **/
public class VisionAllignmentRumble extends Rumble {
    private double lateralCourseDistance;
    private BlockDetectionState blockDetectionState;
    private LeftSideRumble leftSideRumble;
    private RightSideRumble rightSideRumble;
    private IntakeState currentIntakeState;

    /**
     Constructor for VisionAllignmentRumble class
        @param manipulatorGamepad Gamepad for manipulator
        @param robotState The robot state to use
     **/
    public VisionAllignmentRumble(GamepadEx manipulatorGamepad, RobotState robotState, double lateralCourseDistance) {
        super(manipulatorGamepad, robotState);
        this.lateralCourseDistance = lateralCourseDistance;

        rightSideRumble = new RightSideRumble(manipulatorGamepad, robotState);
        leftSideRumble = new LeftSideRumble(manipulatorGamepad, robotState);
    }


    // Check if the block is detected by the robot for vision allignment
    @Override
    public void updateRumble() {
        if (blockDetectionState == BlockDetectionState.DETECTED) {
            if (currentIntakeState == IntakeState.PREPARE_TO_PICKUP) {
                runRumble();
            }
        }
    }

    // TODO: Add a distance-intensity based rumble (as the robot gets closer to the block, the intensity of the rumble increases)
    @Override
    public void runRumble() {
        lateralCourseDistance = robotState.getBlockLateralCoarse();
        if (blockDetectionState == BlockDetectionState.DETECTED) {
            if ((int) lateralCourseDistance > 0) {
                rightSideRumble.runRumble();
            } else {
                leftSideRumble.runRumble();
                }
            }
        }
}


