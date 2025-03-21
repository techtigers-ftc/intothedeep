package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.controller.ClimbReminderRumble;
import org.firstinspires.ftc.teamcode.controller.EndgameRumble;
import org.firstinspires.ftc.teamcode.controller.FailedPickupRumble;
import org.firstinspires.ftc.teamcode.controller.Rumble;
import org.firstinspires.ftc.teamcode.controller.VisionAllignmentRumble;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import java.util.ArrayList;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem to control the rumbles and LEDs of the controllers
 */
public class ControllerSubsystem extends CloseableSubsystem {
    private final ArrayList<Rumble> rumbles;
    private final RobotState robotState;
    private final GamepadEx driverGamepad;
    private final GamepadEx manipulatorGamepad;
    private final double lateralCourseDistance;

    /**
     * Constructor for the ControllerSubsystem
     *
     * @param driverGamepad   The first gamepad
     * @param manipulatorGamepad   The second gamepad
     * @param robotState The robot state to use
     */
    public ControllerSubsystem(GamepadEx driverGamepad, GamepadEx manipulatorGamepad, RobotState robotState,
                               double lateralCourseDistance) {
        this.robotState = robotState;
        this.driverGamepad = driverGamepad;
        this.manipulatorGamepad = manipulatorGamepad;
        this.lateralCourseDistance = robotState.getBlockLateralCoarse();

        rumbles = new ArrayList<>();
        rumbles.add(new EndgameRumble(driverGamepad, manipulatorGamepad, robotState));
        rumbles.add(new ClimbReminderRumble(driverGamepad, manipulatorGamepad, robotState));
        rumbles.add(new FailedPickupRumble(manipulatorGamepad, robotState));
        rumbles.add(new VisionAllignmentRumble(manipulatorGamepad, robotState, lateralCourseDistance));
    }

    @Override
    public void periodic() {
        handleRumbleEffects();
        handleLEDs();
    }

    /**
     * Handles the rumble effects for the controllers
     */
    private void handleRumbleEffects() {
        for (Rumble rumble : rumbles) {
            rumble.updateRumble();
        }
    }

    /**
     * Sets the led colors based on the vision color preference
     */
    private void handleLEDs() {
        switch (robotState.getBlockColorPreference()) {
            case ALLIANCE:
                if(robotState.isBlue()){
                    driverGamepad.gamepad.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                    manipulatorGamepad.gamepad.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                } else {
                    driverGamepad.gamepad.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                    manipulatorGamepad.gamepad.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                }
                break;
            case YELLOW:
                driverGamepad.gamepad.setLedColor(1, 0.8, 0, Gamepad.LED_DURATION_CONTINUOUS);
                manipulatorGamepad.gamepad.setLedColor(1, 0.8, 0, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            case ANY:
                driverGamepad.gamepad.setLedColor(1, 1, 1, Gamepad.LED_DURATION_CONTINUOUS);
                manipulatorGamepad.gamepad.setLedColor(1, 1, 1, Gamepad.LED_DURATION_CONTINUOUS);
                break;
        }
    }
}
