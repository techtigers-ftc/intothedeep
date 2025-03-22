package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command used to ascend the robot up one level
 */
public class AscendOneLevelCommand extends CommandBase {
    private final RobotState robotState;
    private final AscentSubsystem ascent;
    private final DropperSubsystem dropper;
    private final DriveSubsystem drive;

    /**
     * Constructs a new AscendOneLevelCommand
     *
     * @param robotState the state of the robot
     * @param ascent     the ascent subsystem
     * @param dropper    the dropper subsystem
     * @param drive      the drive subsystem
     */
    public AscendOneLevelCommand(RobotState robotState,
                                 AscentSubsystem ascent,
                                 DropperSubsystem dropper,
                                 DriveSubsystem drive) {
        this.robotState = robotState;
        this.ascent = ascent;
        this.dropper = dropper;
        this.drive = drive;

        addRequirements(ascent, dropper, drive);
    }

    @Override
    public void execute() {
        dropper.leftSlideMotor.setPower(-1);
        dropper.rightSlideMotor.setPower(-1);
        drive.backLeft.setPower(1);
        drive.backRight.setPower(1);
        drive.frontLeft.setPower(1);
        drive.frontRight.setPower(1);


        if (robotState.getIsAscending()
                && dropper.getCurrentSlidePositionInches() < AscentSubsystem.JACKS_SLIDES_DISENGAGE_HEIGHT
                && ascent.areJacksEngaged()) {
            ascent.disengageJacks();
        }
    }

    @Override
    public boolean isFinished() {
        return dropper.getCurrentSlidePositionInches() < AscentSubsystem.JOSH_FAILSAFE_LIMIT;
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
}
