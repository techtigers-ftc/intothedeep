package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.AutoDropperForwardCarryAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.core.display.Color;

/**
 * Drives to the last specimen drop, to bring out the intake after the robot reaches past a certain point
 */
public class DriveToLastGeneralSpecimenDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToLastGeneralSpecimenDropState.class.getSimpleName();

    /**
     * Constructor for the DriveToLastSpecimenDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     * @param intake     The intake subsystem
     */
    public DriveToLastGeneralSpecimenDropState(String name, DriveSubsystem drive, DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        super(name, drive, robotState, 3.5);
        addCommands(
                autoDriveCommand,
                new AutoDropperForwardCarryAction(dropper, robotState),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getX() < 88),
                        new IntakeWristRotationAction(intake,
                                IntakeSubsystem.WRIST_ROTATION_READY_TO_PICKUP_POSITION, 0),
                        new IntakeWristPitchAction(intake,
                                IntakeSubsystem.WRIST_PITCH_READY_TO_PICKUP_POSITION, 0),
                        new IntakeOpenAction(intake, 0)
                )
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        robotState.setDebugColor(Color.BLACK);
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperState() == DropperState.FORWARD_CARRY) {
            return AutoState.DRIVE_END;
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        }
        return AutoState.RUNNING;
    }
}
