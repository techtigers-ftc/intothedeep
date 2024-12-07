package org.firstinspires.ftc.teamcode.commands.actions.dropper;

import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;

import team.techtigers.base.actions.ServoActionCommand;

/**
 * Action Command to move the dropper pitch to a certain position
 */
public class DropperPitchAction extends ServoActionCommand {
    private final DropperSubsystem dropper;

    /**
     * Initializes the command with the dropper subsystem, the expected servo position, and the duration
     *
     * @param dropper          the dropper subsystem
     * @param expectedServoPos the expected servo position
     * @param duration         the duration of the command
     */
    public DropperPitchAction(DropperSubsystem dropper,
                              double expectedServoPos, long duration) {
        super(expectedServoPos, duration);
        this.dropper = dropper;
    }

    @Override
    protected double getPosition() {
        return dropper.getPitch();
    }

    @Override
    protected void setPosition(double position) {
        dropper.setPitchAbsolute(position);
    }
}
