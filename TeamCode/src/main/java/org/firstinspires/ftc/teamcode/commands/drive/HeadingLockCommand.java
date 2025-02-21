package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.paths.Waypoint;

/**
 * A command that adjusts the robot to be at a certain angle to intake or drop pixels, and keeps that
 * heading while strafing or driving.
 */

public class HeadingLockCommand extends CommandBase {
    private static final PIDCoefficients ROTATIONAL_COEFFICIENTS =
            new PIDCoefficients(0.5, 0, 0.1);
    private final DriveSubsystem subsystem;
    private final GamepadEx gamepad;
    private final RobotState robotState;
    private final PIDController rotationalController;
    private double targetHeading;

    /**
     * Initializes a new HeadingLockAssistCommand
     *
     * @param subsystem  The drive subsystem to be used.
     * @param robotState The robot state, used to get the odometry values
     * @param gamepad    Gamepad used to get the forward power for driving
     */
    public HeadingLockCommand(DriveSubsystem subsystem,
                                    RobotState robotState,
                                    GamepadEx gamepad) {
        this.robotState = robotState;
        this.subsystem = subsystem;
        this.gamepad = gamepad;
        targetHeading = Math.toRadians(90);

        rotationalController = new PIDController(
                ROTATIONAL_COEFFICIENTS.p, ROTATIONAL_COEFFICIENTS.i, ROTATIONAL_COEFFICIENTS.d);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        rotationalController.reset();
        rotationalController.setTolerance(Math.toRadians(1));
    }

    /*
     * Converts from 0 - 2PI to -PI to PI range
     */
    private double convertHeading(double heading) {
        heading = (heading + 3 * Math.PI) % (2 * Math.PI);

        return heading;
    }

    @Override
    public void execute() {
        Waypoint currentPose = robotState.getRobotCurrentPose();

        // Calculate heading power
        double headingDiff = convertHeading(targetHeading - currentPose.getHeading());
        // NOTE: Using heading diff in the spot of set point in order for the error to be calculated
        // correctly, as we convert the heading to -pi to pi range
        double headingPower = rotationalController.calculate(0, headingDiff);

        // Compensate forward power for change to field centric
        double strafePower = gamepad.getLeftX();
        double forwardPower = gamepad.getLeftY();

        subsystem.driveRobotCentric(
                forwardPower,
                strafePower,
                headingPower
        );
    }
}