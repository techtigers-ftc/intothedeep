package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import java.util.function.DoubleSupplier;

/**
 * Command to move intake to observation zone, dropping its block in the zone
 * ans moving to ready to intake
 */
public class IntakeToObservationZoneAction extends ParallelCommandGroup {
    private static final String LOG_TAG = IntakeToObservationZoneAction.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Creates a new IntakeToPrepareToIntakeCommand
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeToObservationZoneAction(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addRequirements(intake);
        addCommands(
                new IntakeSlidesAbsoluteAction(intake, () -> 10, 1),
                new IntakeWristRotationAction(intake,
                        IntakeSubsystem.WRIST_ROTATION_READY_TO_PICKUP_POSITION, 500),
                new IntakeClawRotationAction(intake,
                        () -> IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION, 500),
                new IntakeWristPitchAction(intake,
                        IntakeSubsystem.WRIST_PITCH_READY_TO_PICKUP_POSITION, 500)
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
            robotState.setIntakeState(IntakeState.READY_TO_PICKUP);
            if(robotState.getBlockPosition() == RobotBlockPosition.INTAKE){
                robotState.setBlockPosition(RobotBlockPosition.NONE);
            }
            robotState.setCurrentGear(DriveGears.ENGAGED);
        }
    }
}
