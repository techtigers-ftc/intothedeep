package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFineAlignAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;

/**
 * A state that aligns over the first colored sample while dropping a colored sample into the observation zone
 */
public class IntakeAndDropFirstColoredSampleState extends ParallelCommandGroupState<AutoState> {
    RobotState robotState;

    /**
     * Constructs a new IntakeAndDropFirstColoredSampleState
     * @param name the name of the state
     * @param drive the drive subsystem
     * @param intake the intake subsystem
     * @param dropper the dropper subsystem
     * @param robotState the robot state
     */
    public IntakeAndDropFirstColoredSampleState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name);
        addCommands(
            new SequentialCommandGroup(
                    new WaitUntilCommand(robotState::hasBlockBeenDetected),
                    new IntakeFineAlignAction(drive, intake, () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()), robotState)
            ),
            new DropperOpenAction(dropper, 100)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER && robotState.getDropperClawState() == ClawState.OPEN) {
            return AutoState.SAMPLE_INTAKE_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
