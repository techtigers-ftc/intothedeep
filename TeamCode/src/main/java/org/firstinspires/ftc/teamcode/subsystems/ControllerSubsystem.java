package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.controller.ClimbReminderRumble;
import org.firstinspires.ftc.teamcode.controller.ColorPreferenceLEDEffect;
import org.firstinspires.ftc.teamcode.controller.EndgameRumble;
import org.firstinspires.ftc.teamcode.controller.FailedPickupRumble;
import org.firstinspires.ftc.teamcode.controller.ControllerEffect;
import org.firstinspires.ftc.teamcode.controller.SwitchIntakeModeRumble;
import org.firstinspires.ftc.teamcode.controller.ToggleBreakBeamRumble;
import org.firstinspires.ftc.teamcode.controller.VisionAlignmentRumble;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import java.util.ArrayList;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem to control the rumbles and LEDs of the controllers
 */
public class ControllerSubsystem extends CloseableSubsystem {
    private final ArrayList<ControllerEffect> controllerEffects;
    /**
     * Constructor for the ControllerSubsystem
     *
     * @param driverGamepad   The first gamepad
     * @param manipulatorGamepad   The second gamepad
     * @param robotState The robot state to use
     */
    public ControllerSubsystem(GamepadEx driverGamepad, GamepadEx manipulatorGamepad, RobotState robotState) {

        controllerEffects = new ArrayList<>();
        controllerEffects.add(new EndgameRumble(driverGamepad, robotState));
        controllerEffects.add(new EndgameRumble(manipulatorGamepad, robotState));

        controllerEffects.add(new ClimbReminderRumble(driverGamepad, robotState));
        controllerEffects.add(new ClimbReminderRumble(manipulatorGamepad, robotState));

        controllerEffects.add(new FailedPickupRumble(manipulatorGamepad, robotState));
        controllerEffects.add(new VisionAlignmentRumble(driverGamepad, robotState));
        controllerEffects.add(new ToggleBreakBeamRumble(manipulatorGamepad, robotState));
        controllerEffects.add(new SwitchIntakeModeRumble(manipulatorGamepad, robotState));

        controllerEffects.add(new ColorPreferenceLEDEffect(driverGamepad, robotState));
    }

    @Override
    public void periodic() {
        for (ControllerEffect controllerEffect : controllerEffects) {
            controllerEffect.updateEffect();
        }
    }
}
