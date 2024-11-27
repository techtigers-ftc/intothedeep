package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command to drive the robot during teleop using Pedro Pathing's teleop enhancements
 */
public class PedroManualDriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final Follower follower;
    private final RobotState robotState;
    private final GamepadEx driverGamepad;

    /**
     * Constructs a new PedroManualDriveCommand
     * @param drive the drive subsystem
     * @param robotState the robot state
     */
    public PedroManualDriveCommand(DriveSubsystem drive, RobotState robotState, GamepadEx driverGamepad) {
        this.robotState = robotState;
        this.drive = drive;
        this.follower = new Follower(robotState);
        this.driverGamepad = driverGamepad;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        follower.startTeleopDrive();
    }

    @Override
    public void execute() {
        follower.setTeleOpMovementVectors(-driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX());
        follower.update();
        drive.drivePedroPath(follower.getCurrentDriveVectors());
    }
}
