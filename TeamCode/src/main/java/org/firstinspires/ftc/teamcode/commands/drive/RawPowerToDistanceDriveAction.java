package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commands.TimeoutCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * An action which drives the robot forward/backward at a specific power until distance
 */
public class RawPowerToDistanceDriveAction extends CommandBase {
    private DriveSubsystem drive;
    private double power;
    private RobotState robotState;
    private double distance;

    /**
     * Constructor for RawPowerDriveAction
     *
     * @param drive the drive subsystem to drive the robot
     * @param power the power to drive the robot at (-1 to 1)
     * @param distance how close the robot should be from the wall in inches
     */
    public RawPowerToDistanceDriveAction(DriveSubsystem drive, RobotState robotState, double power, double distance) {
        this.drive = drive;
        this.power = power;
        this.robotState = robotState;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        robotState.setRunDistanceSensor(true);
    }

    @Override
    public void execute() {
        drive.driveRobotCentric(power, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return robotState.getDistanceSensorValue() < distance;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotCentric(0, 0, 0);
        robotState.setRunDistanceSensor(false);
    }
}
