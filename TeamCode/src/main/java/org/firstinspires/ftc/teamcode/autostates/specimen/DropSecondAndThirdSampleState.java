package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;

import java.util.function.DoubleSupplier;

/**
 * A state to grab a sample to be dropped off later
 */
public class DropSecondAndThirdSampleState extends HoldPointStateBase {
    private static final String LOG_TAG =
            DropSecondAndThirdSampleState.class.getSimpleName();
    private final RobotState robotState;


    /**
     * Constructor for the GrabSampleState
     *
     * @param name       The name of the state
     * @param intake     The intake subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DropSecondAndThirdSampleState(String name, IntakeSubsystem intake, DropperSubsystem dropper,
                                         RobotState robotState, DriveSubsystem drive, DoubleSupplier targetSlidePos) {
        super(name, drive, robotState);
        this.robotState = robotState;
        addCommands(
                holdPointCommand,
                new SequentialCommandGroup(
                        new IntakeReadyToTransferAction(intake, robotState),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new DropperTransferAction(dropper, intake, robotState),
                                        new DropperPitchAction(dropper, 300, 500),
                                        new WaitCommand(100),
                                        new DropperOpenAction(dropper, 100)
                                ),
                                new IntakePrepareToPickupAction(intake, dropper, robotState, targetSlidePos)
                        )
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
        if (robotState.getDropperClawState() == ClawState.OPEN) {
            return AutoState.SAMPLE_0_DROP_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
