package org.firstinspires.ftc.teamcode.commands.actions.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Command for canceling all drive movements
 */
public class CancelDriveCommand extends CommandBase {
    private final DriveSubsystem drive;

    /**
     * Creates a new CancelDriveCommand
     * @param drive the drive subsystem
     */
    public CancelDriveCommand(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void end(boolean interruptible) {
        drive.driveRobotCentric(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
