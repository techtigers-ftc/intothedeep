package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.utils.GearSelection;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Commamnd used to manually drive the robot using TeleOp Enhancements from Pedro Pathing
 */
public class PedroManualDriveCommand extends CommandBase {
    private final Follower follower;
    private final DriveSubsystem drive;
    private final RobotState robotState;
    private final GamepadEx gamepad;

    public PedroManualDriveCommand(DriveSubsystem drive, RobotState robotState, Follower follower, GamepadEx gamepad) {
        this.drive = drive;
        this.robotState = robotState;
        this.gamepad = gamepad;
        this.follower = follower;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        follower.startTeleopDrive();
    }

    @Override
    public void execute() {
        double multiplier = robotState.getCurrentGearSelection() == GearSelection.FAST ? 1 : 0.5;
        double turnMultiplier = robotState.getCurrentGearSelection() == GearSelection.FAST ? 0.5 : 0.8;
        follower.setTeleOpMovementVectors(-gamepad.getLeftY() * multiplier,
                -gamepad.getLeftX() * multiplier,
                -gamepad.getRightX() * turnMultiplier);
        drive.driveFollower(follower);
    }
}