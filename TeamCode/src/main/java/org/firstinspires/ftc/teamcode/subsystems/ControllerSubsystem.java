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
    private final GamepadEx gamepad1;
    private final GamepadEx gamepad2;
    private final double lateralCourseDistance;

    /**
     * Constructor for the ControllerSubsystem
     *
     * @param gamepad1   The first gamepad
     * @param gamepad2   The second gamepad
     * @param robotState The robot state to use
     */
    public ControllerSubsystem(GamepadEx gamepad1, GamepadEx gamepad2, RobotState robotState,
                               double lateralCourseDistance) {
        this.robotState = robotState;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.lateralCourseDistance = robotState.getBlockLateralCoarse();

        rumbles = new ArrayList<>();
        rumbles.add(new EndgameRumble(gamepad1, gamepad2, robotState));
        rumbles.add(new ClimbReminderRumble(gamepad1, gamepad2, robotState));
        rumbles.add(new FailedPickupRumble(gamepad2, robotState));
        rumbles.add(new VisionAllignmentRumble(gamepad2, robotState, lateralCourseDistance));
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
                    gamepad1.gamepad.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                    gamepad2.gamepad.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                } else {
                    gamepad1.gamepad.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                    gamepad2.gamepad.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                }
                break;
            case YELLOW:
                gamepad1.gamepad.setLedColor(1, 0.8, 0, Gamepad.LED_DURATION_CONTINUOUS);
                gamepad2.gamepad.setLedColor(1, 0.8, 0, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            case ANY:
                gamepad1.gamepad.setLedColor(1, 1, 1, Gamepad.LED_DURATION_CONTINUOUS);
                gamepad2.gamepad.setLedColor(1, 1, 1, Gamepad.LED_DURATION_CONTINUOUS);
                break;
        }
    }
}
