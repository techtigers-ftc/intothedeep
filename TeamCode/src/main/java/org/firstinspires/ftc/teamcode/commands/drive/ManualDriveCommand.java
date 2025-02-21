package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command to drive the robot during teleop using Pedro Pathing's teleop enhancements
 */
public class ManualDriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final RobotState robotState;
    private final GamepadEx driverGamepad;

    /**
     * Constructs a new PedroManualDriveCommand
     *
     * @param drive         the drive subsystem
     * @param robotState    the robot state
     * @param driverGamepad the driver gamepad
     */
    public ManualDriveCommand(DriveSubsystem drive,
                              RobotState robotState, GamepadEx driverGamepad) {
        this.drive = drive;
        this.robotState = robotState;
        this.driverGamepad = driverGamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (!robotState.isVisionAligning()) {
            drive.driveRobotCentric(driverGamepad.getLeftY(),
                    driverGamepad.getLeftX(), driverGamepad.getRightX());
        }
    }
}
