package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.SlidingAverageCalculator;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem that controls the robot's sensors.
 */
public class SensorSubsystem extends CloseableSubsystem {
    private final RobotState robotState;
    private final VoltageSensor voltage;
    private final SlidingAverageCalculator voltageAverage;
    private final DistanceSensor distanceSensor;

    /**
     * Constructs a new SensorSubsystem.
     *
     * @param hardwareMap The hardware map, used to get hardware references
     * @param robotState The robot state, used to get the robot's current state
     */
    public SensorSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        voltage = hardwareMap.voltageSensor.iterator().next();
        voltageAverage = new SlidingAverageCalculator(200);
        voltageAverage.clear();
        distanceSensor = hardwareMap.get(DistanceSensor.class, "back_distance_sensor");
    }

    @Override
    public void periodic() {
        voltageAverage.add(voltage.getVoltage());
        robotState.setVoltage(voltageAverage.getAverage());
        if(robotState.isRunDistanceSensor()){
            robotState.setDistanceSensorValue(distanceSensor.getDistance(DistanceUnit.INCH));
        }
    }

    /**
     * Updates the robotState value of the distance detected by the back distance sensor
     * @return the distance detected by the back distance sensor
     */
    public double updateDistance(){
        robotState.setDistanceSensorValue(distanceSensor.getDistance(DistanceUnit.INCH));
        return robotState.getDistanceSensorValue();
    }
}
