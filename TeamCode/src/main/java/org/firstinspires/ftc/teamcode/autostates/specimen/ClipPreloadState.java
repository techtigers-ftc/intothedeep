package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.drive.RawPowerDriveAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to clip the preload specimen to the chamber
 */
public class ClipPreloadState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            ClipPreloadState.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Constructor for the ClipPreloadState
     *
     * @param name    The name of the state
     * @param dropper The dropper subsystem
     */
    public ClipPreloadState(String name, DropperSubsystem dropper, DriveSubsystem drive, RobotState robotState) {
        super(name);
        this.robotState = robotState;
        addCommands(
                new RawPowerDriveAction(drive, 0.4, 0.1),
                new DropperPitchAction(dropper,
                        DropperSubsystem.PITCH_FRONT_SLAP_POSITION, 0),
                new WaitCommand(100),
                new DropperOpenAction(dropper)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getDropperClawState() == ClawState.OPEN) {
            return AutoState.SPECIMEN_PRELOAD_DROP_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
