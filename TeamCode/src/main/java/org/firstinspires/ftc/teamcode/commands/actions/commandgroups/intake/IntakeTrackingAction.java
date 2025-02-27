package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commands.TimeoutCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

/**
 * Command to move the slides until the small camera sees the block is in the right place
 */
@Config
public class IntakeTrackingAction extends CommandBase {
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    private double frameCount;

    /**
     * Constructs a new IntakeTrackingAction
     *
     * @param intake     the intake subsystem
     * @param robotState robot state
     */
    public IntakeTrackingAction(IntakeSubsystem intake, RobotState robotState) {
        addRequirements(intake);
        this.intake = intake;
        this.robotState = robotState;
        frameCount = 0;
    }

    @Override
    public void initialize() {
        intake.setDirectControl(true);
        robotState.setIntakeTracking(true);
    }

    @Override
    public void execute() {
        if (robotState.getFineBlockDetectionState() == BlockDetectionState.DETECTED) {
            frameCount++;
            intake.setMotorPower(0.15);
        } else {
            frameCount = 0;
            intake.setMotorPower(0.35);
        }
    }

    @Override
    public boolean isFinished() {
        return robotState.getFineBlockDetectionState() == BlockDetectionState.DETECTED && frameCount > 3;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotorPower(0);
        intake.setDirectControl(false);
        intake.moveSlidesRelative(0);
        robotState.setIntakeTracking(false);
    }
}
