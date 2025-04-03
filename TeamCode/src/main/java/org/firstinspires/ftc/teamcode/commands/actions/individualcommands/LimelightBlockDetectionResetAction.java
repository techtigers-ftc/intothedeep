package org.firstinspires.ftc.teamcode.commands.actions.individualcommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

/**
 * Resets the absolute block detection cached in the limelight
 */
public class LimelightBlockDetectionResetAction extends InstantCommand {
    private static final String LOG_TAG = LimelightBlockDetectionResetAction.class.getSimpleName();
    private final LimelightSubsystem limelight;

    /**
     * Initializes the command
     *
     * @param limelight  the limelight subsystem
     */
    public LimelightBlockDetectionResetAction(LimelightSubsystem limelight) {
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        limelight.resetAbsoluteBlockDetection();
    }
}
