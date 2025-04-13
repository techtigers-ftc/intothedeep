package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command used to move the slides while the ascent is engaged.
 */
public class MoveAscentSlidesCommand extends CommandBase {
    private final RobotState robotState;
    private final AscentSubsystem ascent;
    private final DropperSubsystem dropper;
    private final DriveSubsystem drive;
    private final double targetPosition;
    private boolean stopRequested;

    /**
     * Constructs a new MoveAscentSlidesCommand
     *
     * @param robotState     the state of the robot
     * @param ascent         the ascent subsystem
     * @param dropper        the dropper subsystem
     * @param drive          the drive subsystem
     * @param targetPosition the target position for the slides
     */
    public MoveAscentSlidesCommand(RobotState robotState,
                                   AscentSubsystem ascent,
                                   DropperSubsystem dropper,
                                   DriveSubsystem drive,
                                   double targetPosition) {
        this.robotState = robotState;
        this.ascent = ascent;
        this.dropper = dropper;
        this.drive = drive;
        this.targetPosition = targetPosition;
        stopRequested = false;

        addRequirements(ascent, dropper, drive);
    }

    @Override
    public void initialize() {
        stopRequested = false;
    }

    @Override
    public void execute() {
        if (dropper.getCurrentSlidePositionInches() < targetPosition) {
            dropper.leftSlideMotor.setPower(1);
            dropper.rightSlideMotor.setPower(1);
            drive.backLeft.setPower(-1);
            drive.backRight.setPower(-1);
            drive.frontLeft.setPower(-1);
            drive.frontRight.setPower(-1);
        } else {
            dropper.leftSlideMotor.setPower(-1);
            dropper.rightSlideMotor.setPower(-1);
            drive.backLeft.setPower(1);
            drive.backRight.setPower(1);
            drive.frontLeft.setPower(1);
            drive.frontRight.setPower(1);
        }


        if (robotState.isAscending()
                && dropper.getCurrentSlidePositionInches() < AscentSubsystem.JACKS_SLIDES_DISENGAGE_HEIGHT
                && ascent.areJacksEngaged()) {
            ascent.disengageJacks();
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(dropper.getCurrentSlidePositionInches() - targetPosition) < 1 || stopRequested;
    }

    @Override
    public void end(boolean interrupted) {
        dropper.leftSlideMotor.setPower(0);
        dropper.rightSlideMotor.setPower(0);
        drive.backLeft.setPower(0);
        drive.backRight.setPower(0);
        drive.frontLeft.setPower(0);
        drive.frontRight.setPower(0);
    }

    /**
     * Allows for an external command to request the stop of this command
     */
    public void stop() {
        stopRequested = true;
    }
}
