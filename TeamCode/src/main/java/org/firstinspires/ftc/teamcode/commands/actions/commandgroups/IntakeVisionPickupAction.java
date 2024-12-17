package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * A command group that automically picks up a specimen using the vision system and limelight
 */
public class IntakeVisionPickupAction extends SequentialCommandGroup {
    private final RobotState robotState;

    /**
     * Creates a new IntakeToTransferAction
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeVisionPickupAction(IntakeSubsystem intake, RobotState robotState) {
        this.robotState = robotState;
        addCommands();
    }

    @Override
    public void end(boolean interrupted) {
        robotState.setIntakeState(IntakeState.TRANSFER);
        robotState.setBlockPosition(RobotBlockPosition.INTAKE);

    }
}
