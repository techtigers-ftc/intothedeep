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
public class VisionAllignmentRumble extends Rumble {
    private IntakeState currentIntakeState;

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
        if (robotState.hasBlockBeenDetected() && intaking) {
            runRumble();
        }
    }

    @Override
    public void runRumble() {
        Vector robotI = new Vector(1, robotState.getRobotCurrentPose().getHeading());
        Vector robotJ = new Vector(1, robotState.getRobotCurrentPose().getHeading() + Math.PI / 2);
        RobotLog.dd("Vision Rumble", "i hat: " + robotI + " j hat: " + robotJ);
        double lateral = robotState.getAbsoluteBlockCoordinates().getX() * robotI.getXComponent()
                + robotState.getAbsoluteBlockCoordinates().getY() * robotJ.getXComponent();
        RobotLog.dd("Vision Rumble", "lateral: " + lateral);
        if (lateral < 0) {
            gamepad1.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(0, calculateRumbleIntensity(lateral), 50)
                    .build());
        } else {
            gamepad1.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(calculateRumbleIntensity(lateral), 0, 50)
                    .build());
        }
    }

    private double calculateRumbleIntensity(double lateral) {
        return 0.5 / (1 + Math.exp(-1.5 * Math.abs(lateral)));
    }
}


