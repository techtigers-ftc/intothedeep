package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.AutoDropperForwardCarryAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.core.display.Color;

/**
 * Drives to general specimen drop
 */
public class DriveToGeneralSpecimenDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToGeneralSpecimenDropState.class.getSimpleName();
    private final ElapsedTime timer;

    /**
     * Constructor for the DriveToGeneralSpecimenDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DriveToGeneralSpecimenDropState(String name, DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState, 3.5);
        timer = new ElapsedTime();
        addCommands(
                autoDriveCommand,
                new AutoDropperForwardCarryAction(dropper, robotState)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        robotState.setDebugColor(Color.BLACK);
        timer.reset();
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperState() == DropperState.FORWARD_CARRY) {
            robotState.setDebugColor(Color.BLUE);
            return AutoState.DRIVE_END;
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            robotState.setDebugColor(Color.GREEN);
            return AutoState.TIMEOUT;
        }
        return AutoState.RUNNING;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
//        RobotLog.dd("Specimen Auto Debug", "General Drop State Time to End: %f", timer.seconds());
    }
}
