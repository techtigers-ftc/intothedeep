package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem that controls the robot's sensors.
 */
public class SensorSubsystem extends CloseableSubsystem {
    private final RobotState robotState;
    private final VoltageSensor voltage;

    /**
     * Constructs a new SensorSubsystem.
     *
     * @param hardwareMap The hardware map, used to get hardware references
     * @param robotState The robot state, used to get the robot's current state
     */
    public SensorSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        voltage = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        robotState.setVoltage(voltage.getVoltage());
    }
}
