package org.firstinspires.ftc.teamcode.commands.actions.individualcommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

/**
 * Sets the limelight lateral bounds to given parameters
 */
public class LimelightLateralBoundsAction extends CommandBase {
    private static final String LOG_TAG = LimelightLateralBoundsAction.class.getSimpleName();
    private final LimelightSubsystem limelight;
    private double lowerBound;
    private double upperBound;

    /**
     * Initializes the command
     *
     * @param limelight  the limelight subsystem
     * @param lowerBound the lower bound to set
     * @param upperBound the upper bound to set
     */
    public LimelightLateralBoundsAction(LimelightSubsystem limelight,
                                        double lowerBound, double upperBound) {
        this.limelight = limelight;
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
    }

    @Override
    public void initialize() {
        limelight.setLateralLowerBound(lowerBound);
        limelight.setLateralUpperBound(upperBound);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
