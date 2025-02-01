package org.firstinspires.ftc.teamcode.commands.actions.individualcommands.rumble;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Action that rumbles to signal a takeover of drive by a different command
 */
public class TakeoverRumbleAction extends CommandBase {
    private GamepadEx gamepad;

    /**
     * Constructs a new TakeoverRumbleAction
     *
     * @param gamepad the gamepad to rumble
     */
    public TakeoverRumbleAction(GamepadEx gamepad) {
        this.gamepad = gamepad;
    }

    @Override
    public void execute() {
        if (gamepad != null && !gamepad.gamepad.isRumbling()) {
            gamepad.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder().addStep(
                    0.5, 0.5, 10000
            ).build());
        }
    }

    @Override
    public boolean isFinished() {
        return gamepad == null;
    }

    @Override
    public void end(boolean interrupted) {
        gamepad.gamepad.stopRumble();
    }
}
