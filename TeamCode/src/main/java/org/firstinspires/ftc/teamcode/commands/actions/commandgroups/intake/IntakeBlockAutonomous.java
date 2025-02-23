package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeCloseAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeLoosenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristRotationAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;
import org.firstinspires.ftc.teamcode.utils.enums.DriveGears;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * Command to move intake to Ready To Transfer without vision
 */
public class IntakeBlockAutonomous extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;
    private final IntakeSubsystem intake;
    private double lastClawRotation;

    /**
     * Creates a new IntakeBlockAutonomous
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     * @param command    the command to cancel
     */
    public IntakeBlockAutonomous(IntakeSubsystem intake,
                                 DropperSubsystem dropper,
                                 RobotState robotState, CommandBase command) {
        this.robotState = robotState;
        this.intake = intake;
        lastClawRotation = 90;
        addRequirements(intake, dropper);
        addCommands(
                new IntakeWristPitchAction(intake, IntakeSubsystem.WRIST_PITCH_PECK_POSITION, 100),
                new IntakeCloseAction(intake, 150),
                new InstantCommand(() -> robotState.setVisionAligning(false)),
                new ParallelCommandGroup(
                        new IntakeClawRotationAction(intake, () -> IntakeSubsystem.CLAW_ROTATION_TRANSFER_POSITION, 100),
                        new IntakeWristPitchAction(intake,
                                IntakeSubsystem.WRIST_PITCH_TRANSFER_POSITION, 200),
                        new DropperTransferAction(dropper, robotState)
                ),
                new ParallelCommandGroup(
                        new IntakeWristRotationAction(intake,
                                IntakeSubsystem.WRIST_ROTATION_TRANSFER_POSITION, 300),
                        new IntakeLoosenAction(intake, 300),
                        new IntakeSlidesAbsoluteAction(intake,
                                () -> IntakeSubsystem.SLIDES_TRANSFER_POSITION, 1)
                ),
                new WaitCommand(100)
        );
    }

    /**
     * Overload constructor for if the command doesn't receive a command to cancel
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public IntakeBlockAutonomous(IntakeSubsystem intake,
                                 DropperSubsystem dropper,
                                 RobotState robotState) {
        this(intake, dropper, robotState, null);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted || robotState.getDropperState() == DropperState.TRANSFER) {
            robotState.setIntakeState(IntakeState.READY_TO_TRANSFER);
            robotState.setBlockPosition(RobotBlockPosition.INTAKE);
            robotState.setCurrentGear(DriveGears.NOT_ENGAGED);
        } else {
            robotState.setIntakeState(IntakeState.READY_TO_PICKUP);
            robotState.setCurrentGear(DriveGears.ENGAGED);
            intake.setWristPitchAbsolute(IntakeSubsystem.WRIST_PITCH_READY_TO_PICKUP_POSITION);
            intake.setWristRotationAbsolute(IntakeSubsystem.WRIST_ROTATION_READY_TO_PICKUP_POSITION);
            intake.openClaw();
            intake.setClawRotationAbsolute(IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION);
            if (!robotState.isManualIntakeSelected()) {
                intake.moveSlidesRelative(-3);
            }
        }
    }
}
