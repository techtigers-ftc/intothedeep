package org.firstinspires.ftc.teamcode.commands.actions.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * An action which drives the robot forward/backward at a specific power for a certain amount of time
 */
public class RawPowerDriveAction extends CommandBase {
    private DriveSubsystem drive;
    private double power;
    private double time;
    private ElapsedTime timer;

    /**
     * Constructor for RawPowerDriveAction
     *
     * @param drive the drive subsystem to drive the robot
     * @param power the power to drive the robot at (-1 to 1)
     * @param time the time to drive the robot for
     */
    public RawPowerDriveAction(DriveSubsystem drive, double power, double time) {
        this.drive = drive;
        this.power = power;
        this.time = time;
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        timer.reset();
        drive.driveRobotCentric(power, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() > time;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotCentric(0, 0, 0);
    }
}
