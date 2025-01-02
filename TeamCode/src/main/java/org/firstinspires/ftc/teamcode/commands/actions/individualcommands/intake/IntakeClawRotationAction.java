package org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import team.techtigers.base.actions.ServoActionCommand;

/**
 * Action Command to move the intake claw rotation to a certain position
 */
public class IntakeClawRotationAction extends ServoActionCommand {
    private final IntakeSubsystem intake;

    /**
     * Initializes the command with the intake subsystem, the expected servo position, and the duration
     *
     * @param intake              the intake subsystem
     * @param expectedPosSupplier the supplier for the expected servo position
     * @param duration            the duration of the command
     */
    public IntakeClawRotationAction(IntakeSubsystem intake,
                                    DoubleSupplier expectedPosSupplier,
                                    long duration) {
        super(expectedPosSupplier, duration);
        this.intake = intake;
    }

    @Override
    protected double getPosition() {
        return intake.getClawRotation();
    }

    @Override
    protected void setPosition(double position) {
        intake.setClawRotationAbsolute(position);
    }
}
