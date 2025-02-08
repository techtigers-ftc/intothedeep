package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import java.util.function.DoubleSupplier;


/**
 * A command used to ascend the robot
 */
public class ManualAscentCommand extends CommandBase {
    // Ascent bottom limit
    private static final double JOSH_FAILSAFE_LIMIT = 0.5;
    private final RobotState robotState;
    private final DoubleSupplier powerSupplier;
    private final AscentSubsystem ascent;
    private final DropperSubsystem dropper;
    private final DriveSubsystem drive;

    /**
     * Constructs a new ManualAscentCommand
     *
     * @param robotState the state of the robot
     * @param powerSupplier the supplier for the power to ascend
     * @param ascent the ascent subsystem, to engage the ascent
     * @param dropper the dropper subsystem, to move the slides
     * @param drive the drive subsystem, to move the drive motors
     */
    public ManualAscentCommand(RobotState robotState,
                               DoubleSupplier powerSupplier,
                               AscentSubsystem ascent, DropperSubsystem dropper,
                               DriveSubsystem drive) {
        this.robotState = robotState;
        this.powerSupplier = powerSupplier;
        this.ascent = ascent;
        this.dropper = dropper;
        this.drive = drive;
        addRequirements(ascent, dropper, drive);
    }

    @Override
    public void initialize() {
        ascent.engageAscent();
        if (dropper.getCurrentSlidePositionInches() < 12) {
            this.cancel();
        }
    }

    @Override
    public void execute() {
        double power = powerSupplier.getAsDouble();
        RobotLog.dd(ManualAscentCommand.class.getSimpleName(), "Raw Power: %f",
                power);
        if (dropper.getCurrentSlidePositionInches() > DropperSubsystem.SLIDE_MAX) {
            power = Math.min(0, power);
        } else if (dropper.getCurrentSlidePositionInches() < JOSH_FAILSAFE_LIMIT) {
            power = Math.max(0, power);
        }
        RobotLog.dd(ManualAscentCommand.class.getSimpleName(), "Limited " +
                        "Power: %f");

        dropper.leftSlideMotor.setPower(power);
        dropper.rightSlideMotor.setPower(power);
        drive.backLeft.setPower(power);
        drive.backRight.setPower(power);

        if (robotState.getIsAscending()
                && dropper.getCurrentSlidePositionInches() < AscentSubsystem.JACKS_SLIDES_DISENGAGE_HEIGHT
                && ascent.areJacksEngaged()) {
            ascent.disengageJacks();
        }
    }
}
