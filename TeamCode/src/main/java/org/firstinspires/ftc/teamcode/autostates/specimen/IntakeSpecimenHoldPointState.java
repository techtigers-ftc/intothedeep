package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autostates.HoldPointStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import java.util.function.DoubleSupplier;

/**
 * A state to pick up the three specimens on the floor while holding the robot position
 */
public class IntakeSpecimenHoldPointState extends HoldPointStateBase {
    private static final String LOG_TAG =
            IntakeSpecimenHoldPointState.class.getSimpleName();
    private final RobotState robotState;
    private int runCounter;

    /**
     * Constructor for the IntakeSpecimenHoldPoint state
     *
     * @param name                  The name of the state
     * @param intake                The intake subsystem
     * @param dropper               The dropper subsystem
     * @param drive                 The drive subsystem
     * @param slidePositionSupplier the supplier for slide position
     * @param robotState            The robot state
     */
    public IntakeSpecimenHoldPointState(String name, IntakeSubsystem intake, DropperSubsystem dropper, DriveSubsystem drive, DoubleSupplier slidePositionSupplier,
                                        RobotState robotState) {
        super(name, drive, robotState);
        this.robotState = robotState;
        runCounter = 0;
        DoubleSupplier firstMoveSupplier = () -> slidePositionSupplier.getAsDouble() - 8;
        addCommands(
                holdPointCommand,
                new SequentialCommandGroup(
                        new IntakeReadyToPickupAction(intake, robotState, firstMoveSupplier, () -> 90),
                        new IntakeSlidesAbsoluteAction(intake, slidePositionSupplier, 1),
                        new IntakePrepareToTransferAction(intake, dropper, () -> 5, robotState)
                )
        );
    }

    /**
     * Initialize the state, incrementing the run counter
     */
    @Override
    public void initialize() {
        runCounter++;
        super.initialize();
    }

    /**
     * Get the current condition of the robot
     *
     * @return the current condition of the robot using the AutoState enum
     */
    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER
                && robotState.getBlockPosition() == RobotBlockPosition.INTAKE) {
            if (runCounter == 1) {
                return AutoState.SPECIMEN_1_INTAKE_COMPLETE;
            } else if (runCounter == 2) {
                return AutoState.SPECIMEN_2_INTAKE_COMPLETE;
            } else if (runCounter == 3) {
                return AutoState.SPECIMEN_3_INTAKE_COMPLETE;
            }
            return AutoState.SPECIMEN_4_INTAKE_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
