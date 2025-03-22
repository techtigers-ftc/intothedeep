package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Vector;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

/**
 * A rumble that tells the manipulator which side of the block is relative to the robot
 **/
public class VisionAlignmentRumble extends Rumble {
    private IntakeState currentIntakeState;

    /**
     * Constructor for VisionAlignmentRumble class
     *
     * @param manipulatorGamepad Gamepad for manipulator
     * @param robotState         The robot state to use
     **/
    public VisionAlignmentRumble(GamepadEx manipulatorGamepad, RobotState robotState) {
        super(manipulatorGamepad, robotState);
    }


    /**
     * Rumbles if the robot is intaking and a block is detected in the direction of the block
     */
    @Override
    public void updateRumble() {
        boolean intaking = robotState.getIntakeState() == IntakeState.PREPARE_TO_PICKUP
                || robotState.getIntakeState() == IntakeState.READY_TO_PICKUP;
        if (robotState.hasBlockBeenDetected() && intaking) {
            runRumble();
        }
    }

    /**
     * Runs the rumble effect in the direction of the block
     */
    @Override
    public void runRumble() {
        Vector robotI = new Vector(1, robotState.getRobotCurrentPose().getHeading());
        Vector robotJ = new Vector(1, robotState.getRobotCurrentPose().getHeading() + Math.PI / 2);
        RobotLog.dd("Vision Rumble", "i hat: " + robotI + " j hat: " + robotJ);
        double blockAdjustedX = robotState.getAbsoluteBlockCoordinates().getX() * robotI.getXComponent()
                + robotState.getAbsoluteBlockCoordinates().getY() * robotJ.getXComponent();
        double robotAdjustedX = robotState.getRobotCurrentPose().getX() * robotI.getXComponent()
                + robotState.getAbsoluteBlockCoordinates().getY() * robotJ.getXComponent();
        double lateral = robotAdjustedX - blockAdjustedX;
        RobotLog.dd("Vision Rumble", "lateral: " + lateral);
        if (lateral > 0) {
            gamepad1.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(0, lateral > 0.75 ? 1 : 0, 50)
                    .build());
        } else {
            gamepad1.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(lateral < -0.75 ? 0.5 : 0, 0, 50)
                    .build());
        }
    }
}


