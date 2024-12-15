package org.firstinspires.ftc.teamcode.commands.actions.intake;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import team.techtigers.base.actions.ServoActionCommand;

/**
 * Action Command to move the intake rotation to a certain position
 */
public class IntakeWristRotationAction extends ServoActionCommand {
    private final IntakeSubsystem intake;

    /**
     * Initializes the command with the intake subsystem, the expected servo position, and the duration
     *
     * @param intake           the intake subsystem
     * @param expectedServoPos the expected servo position
     * @param duration         the duration of the command
     */
    public IntakeWristRotationAction(IntakeSubsystem intake,
                                     double expectedServoPos, long duration) {
        super(expectedServoPos, duration);
        this.intake = intake;
    }

    @Override
    protected double getPosition() {
        return intake.getRotation();
    }

    @Override
    protected void setPosition(double position) {
        intake.setWristRotationAbsolute(position);
    }
}
