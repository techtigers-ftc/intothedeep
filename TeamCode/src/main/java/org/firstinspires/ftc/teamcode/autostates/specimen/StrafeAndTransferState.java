package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.TransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFullReadyToTransferNoVisionAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * A state that strafes to the second colored sample while transferring a colored sample to the dropper
 */
public class StrafeAndTransferState extends DriveStateBase {
    /**
     * Constructor for StrafeAndTransferState
     * @param name name of the state
     * @param drive drive subsystem
     * @param intake intake subsystem
     * @param dropper dropper subsystem
     * @param robotState robot state
     * @param timeout timeout for the state
     */
    public StrafeAndTransferState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState, double timeout) {
        super(name, drive, robotState, timeout);
        addCommands(
            autoDriveCommand,
            new SequentialCommandGroup(
                new IntakeFullReadyToTransferNoVisionAction(intake, dropper, robotState),
                new TransferAction(dropper, intake, robotState)
            )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if(super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        } else if(super.getCurrentCondition() == AutoState.DRIVE_END && robotState.getBlockPosition() == RobotBlockPosition.DROPPER) {
            return AutoState.DRIVE_END;
        }
        return AutoState.RUNNING;
    }
}
