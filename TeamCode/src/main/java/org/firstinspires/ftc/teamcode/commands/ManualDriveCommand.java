package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * A command to drive the robot during teleop using Pedro Pathing's teleop enhancements
 */
public class ManualDriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final GamepadEx driverGamepad;

    /**
     * Constructs a new PedroManualDriveCommand
     *
     * @param drive         the drive subsystem
     * @param driverGamepad the driver gamepad
     */
    public ManualDriveCommand(DriveSubsystem drive, GamepadEx driverGamepad) {
        this.drive = drive;
        this.driverGamepad = driverGamepad;
        addRequirements(drive);
    }


    @Override
    public void execute() {
        drive.driveRobotCentric(-driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX());
    }
}
