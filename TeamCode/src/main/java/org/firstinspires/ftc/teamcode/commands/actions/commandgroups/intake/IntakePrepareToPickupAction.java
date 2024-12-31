package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * Command to move intake to prepare to intake state.
 */
public class IntakePrepareToPickupAction extends ParallelCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new IntakeToPrepareToIntakeCommand
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakePrepareToPickupAction(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeSlidesAbsoluteAction(intake, 10, 0.5),
                new IntakeWristRotationAction(intake,
                        IntakeSubsystem.WRIST_ROTATION_READY_TO_PICKUP_POSITION, 500),
                new IntakeClawRotationAction(intake,
                        IntakeSubsystem.CLAW_ROTATION_READY_TO_PICKUP_POSITION, 500),
                new IntakeWristPitchAction(intake,
                        IntakeSubsystem.WRIST_PITCH_PICKUP_POSITION, 500),
                new IntakeOpenAction(intake)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        RobotLog.dd(LOG_TAG, "Executing command from state: %s", robotState.getIntakeState());
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        if (!interrupted) {
            robotState.setIntakeState(IntakeState.PREPARE_TO_PICKUP);
            if(robotState.getBlockPosition() == RobotBlockPosition.INTAKE){
                robotState.setBlockPosition(RobotBlockPosition.NONE);
            }
            robotState.setCurrentGear(DriveGears.ENGAGED);
        }
    }
}
