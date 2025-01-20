package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autostates.HoldPointStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

/**
 * A state to grab a sample to be dropped off later
 */
public class DropSampleState extends HoldPointStateBase {
    private static final String LOG_TAG =
            DropSampleState.class.getSimpleName();
    private final RobotState robotState;


    /**
     * Constructor for the GrabSampleState
     *
     * @param name       The name of the state
     * @param intake     The intake subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DropSampleState(String name, IntakeSubsystem intake, DropperSubsystem dropper,
                           RobotState robotState, DriveSubsystem drive) {
        super(name, drive, robotState);
        this.robotState = robotState;
        addCommands(
                holdPointCommand,
                new SequentialCommandGroup(
                        new IntakeReadyToTransferAction(intake, robotState),
                        new DropperTransferAction(dropper, intake, robotState),
                        new DropperPitchAction(dropper, 300, 700),
                        new DropperOpenAction(dropper, 100)
                )
        );
    }

    /**
     * Get the current condition of the robot
     *
     * @return the current condition of the robot using the AutoState enum
     */
    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperClawState() == ClawState.OPEN &&
                robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER) {
            return AutoState.SAMPLE_DROP_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
