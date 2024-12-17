package org.firstinspires.ftc.teamcode.commands.actions.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Moves the intake claw rotation to the position that is based on the value from the fine camera
 * orientation reading
 */
@Config
public class IntakeFineRotationAction extends CommandBase {
    private final IntakeSubsystem intake;
    private final RobotState robotState;

    /**
     * Initializes the command
     *
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public IntakeFineRotationAction(IntakeSubsystem intake, RobotState robotState) {
        this.intake = intake;
        this.robotState = robotState;
    }

    @Override
    public void execute() {
        intake.setClawRotationAbsolute(90 - robotState.getBlockOrientation());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

