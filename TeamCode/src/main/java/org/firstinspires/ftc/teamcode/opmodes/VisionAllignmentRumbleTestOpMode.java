package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.robot.Robot;
import team.techtigers.core.utils.RobotSaveState;

import org.firstinspires.ftc.teamcode.controller.VisionAllignmentRumble;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

import team.techtigers.base.BaseOpMode;

public class VisionAllignmentRumbleTestOpMode extends BaseOpMode {
    private VisionAllignmentRumble visionAllignmentRumble;
    private final int lateralCourseDistance = 5;
    private GamepadEx manipulatorGamepad;
    private RobotState robotState;
    private boolean isBlockDetected;
    private BlockDetectionState blockDetectionState;

    @Override
    public void initialize() {
        visionAllignmentRumble = new VisionAllignmentRumble(manipulatorGamepad, robotState,
                lateralCourseDistance);
        blockDetectionState = robotState.getCoarseBlockDetectionState();
        if (blockDetectionState == BlockDetectionState.DETECTED) {
            isBlockDetected = true;
        } else {
            isBlockDetected = false;
        }

    }

    public void execute() {
        visionAllignmentRumble.runRumble();
    }

    public void update() {
        telemetry.addData("BLock Lateral Course Distance:", lateralCourseDistance);
        telemetry.addData("Is Block Detected:", isBlockDetected);
    }
}
