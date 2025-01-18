package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import java.util.function.DoubleSupplier;

/**
 * A State to drive to the submersible position
 */
public class SecondDropOff extends DriveStateBase {
    private static final String LOG_TAG =
            SecondDropOff.class.getSimpleName();
    private static final double TOLERANCE = 1.5;
    private static final double ANGULAR_TOLERANCE = Math.toRadians(5);

    /**
     * Constructor for the DriveToSubmersible
     *
     * @param name The name of the state
     * @param drive The drive subsystem
     * @param robotState The robot state
     */
    public SecondDropOff(String name, DriveSubsystem drive, RobotState robotState, IntakeSubsystem intake, DoubleSupplier doubleSupplier) {
        super(name, drive, robotState);
        addCommands(
                autoDriveCommand,
                new IntakeSlidesAbsoluteAction(intake, doubleSupplier, 0.5)

        );
    }
}
