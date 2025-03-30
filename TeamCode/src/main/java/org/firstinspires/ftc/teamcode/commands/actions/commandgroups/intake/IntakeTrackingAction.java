package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

/**
 * Command to creep the slides forward until the Limelight sees the block. This command makes sure
 * the Limelight sees the block for a few frames before picking it up to make sure there is no
 * ghost block detected.
 */
@Config
public class IntakeTrackingAction extends CommandBase {
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    private double frameCount;
    private double detectedSlidePosition;

    private static final double BASE_POWER = 0.275;
    private static final double INCREMENTAL_POWER = 0.005;

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
        detectedSlidePosition = 0;
    }

    @Override
    public void initialize() {
        intake.setDirectControl(true);
        robotState.setIntakeTracking(true);
        robotState.setCoarseCameraMode(false);
    }

    @Override
    public void execute() {
        double power;
        if (robotState.isBlockDetected()) {
            if (frameCount == 0) {
                detectedSlidePosition = intake.getCurrentSlidePositionInches();
            }
            frameCount++;
            power = 0;
        } else {
            frameCount = 0;
            power = BASE_POWER + INCREMENTAL_POWER * intake.getCurrentSlidePositionInches();
        }
        RobotLog.dd("IntakeTrackingAction", "Setting motor power: %f", power);
        RobotLog.dd("IntakeTrackingAction", "Current Slide Extension: %f", intake.getCurrentSlidePositionInches());
        RobotLog.dd("IntakeTrackingAction", "Frame Count: %f", frameCount);
        intake.setMotorPower(power);
    }

    @Override
    public boolean isFinished() {
        return robotState.isBlockDetected() && frameCount > 2;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotorPower(0);
        intake.setDirectControl(false);
        robotState.setIntakeTracking(false);
        if (!interrupted) {
            intake.moveSlidesAbsolute(detectedSlidePosition - 0.5);
        } else {
            intake.moveSlidesRelative(0);
        }
    }
}
