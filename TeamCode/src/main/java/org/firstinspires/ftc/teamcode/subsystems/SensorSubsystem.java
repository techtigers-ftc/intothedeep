package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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
    private final IMU controlHubIMU;

    /**
     * Constructs a new SensorSubsystem.
     *
     * @param hardwareMap The hardware map, used to get hardware references
     * @param robotState  The robot state, used to get the robot's current state
     */
    public SensorSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        voltage = hardwareMap.voltageSensor.iterator().next();
        voltageAverage = new SlidingAverageCalculator(200);
        voltageAverage.clear();
        distanceSensor = hardwareMap.get(DistanceSensor.class, "back_distance_sensor");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        controlHubIMU = hardwareMap.get(IMU.class, "imu");
        controlHubIMU.initialize(parameters);
    }

    @Override
    public void periodic() {
        voltageAverage.add(voltage.getVoltage());
        robotState.setVoltage(voltageAverage.getAverage());
        if (robotState.isRunDistanceSensor()) {
            robotState.setDistanceSensorValue(distanceSensor.getDistance(DistanceUnit.INCH));
        }
        YawPitchRollAngles angles = controlHubIMU.getRobotYawPitchRollAngles();
        robotState.setRobotPitch(angles.getPitch());
        RobotLog.dd(tag, "Yaw: %f, Pitch: %f, Roll: %f", angles.getYaw(),
                angles.getPitch(), angles.getRoll());
    }
}
