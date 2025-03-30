package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Command for canceling all drive movements
 */
public class CancelDriveCommand extends InstantCommand {
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
}
