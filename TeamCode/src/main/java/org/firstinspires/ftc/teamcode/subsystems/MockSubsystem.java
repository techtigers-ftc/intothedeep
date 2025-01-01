package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.RobotState;

import java.util.Arrays;
import java.util.List;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem which saves a detected sample's attributes into RobotState
 */
@Config
public class MockSubsystem extends CloseableSubsystem {
    public static double blockForwardCoarse = 0.0;
    public static double blockLateralCoarse = 0.0;
    public static double blockForwardFine = 0.0;
    public static double blockLateralFine = 0.0;
    public static double blockOrientation = 0.0;
    private final RobotState robotState;
    /**
     * Constructor a subsystem that represents another subsystem by mocking its inputs to state
     *
     * @param robotState  Used to set limelight values in robotstate
     */
    public MockSubsystem(RobotState robotState) {
        this.robotState = robotState;
    }

    @Override
    public void periodic() {
        robotState.setBlockForwardCoarse(blockForwardCoarse);
        robotState.setBlockLateralCoarse(blockLateralCoarse);
        robotState.setBlockForwardFine(blockForwardFine);
        robotState.setBlockLateralFine(blockLateralFine);
        robotState.setBlockOrientation(blockOrientation);
    }
}
