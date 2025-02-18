package org.firstinspires.ftc.teamcode.commands.actions.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.TimeoutCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * An action which drives the robot forward/backward at a specific power for a certain amount of time
 */
public class RawPowerDriveAction extends TimeoutCommand {
    private DriveSubsystem drive;
    private double power;

    /**
     * Constructor for RawPowerDriveAction
     *
     * @param drive the drive subsystem to drive the robot
     * @param power the power to drive the robot at (-1 to 1)
     * @param time the time to drive the robot for
     */
    public RawPowerDriveAction(DriveSubsystem drive, double power, double time) {
        super(time);
        this.drive = drive;
        this.power = power;
    }

    @Override
    public void execute() {
        drive.driveRobotCentric(power, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return isTimeoutReached();
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotCentric(0, 0, 0);
    }
}
