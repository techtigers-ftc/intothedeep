package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.core.display.Color;

/**
 * Drives to general specimen intake
 */
public class DriveToGeneralSpecimenIntakeState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToGeneralSpecimenIntakeState.class.getSimpleName();
    private final ElapsedTime timer;

    /**
     * Constructor for the DriveToGeneralSpecimenIntakeState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DriveToGeneralSpecimenIntakeState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState, 3.5);
        timer = new ElapsedTime();
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new DropperWallIntakeNoTransferAction(dropper, robotState)
                )
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.reset();
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperState() == DropperState.WALL_INTAKE) {
            return AutoState.DRIVE_END;
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        }
        return AutoState.RUNNING;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
//        RobotLog.dd("Specimen Auto Debug", "General Intake State Time to End: %f", timer.seconds());
    }
}
