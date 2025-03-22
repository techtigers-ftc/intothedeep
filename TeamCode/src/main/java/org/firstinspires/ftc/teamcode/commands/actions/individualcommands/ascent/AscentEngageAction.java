package org.firstinspires.ftc.teamcode.commands.actions.individualcommands.ascent;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Engages the ascent
 */
public class AscentEngageAction extends CommandBase {
    private final AscentSubsystem ascent;
    private final ElapsedTime timer;
    private final RobotState robotState;
    private final double waitTime;

    /**
     * Constructs a new AscentEngageAction
     *
     * @param ascent the ascent subsystem
     * @param waitTime the time to wait before finishing
     * @param robotState the robot state
     */
    public AscentEngageAction(AscentSubsystem ascent,
                              RobotState robotState, double waitTime) {
        this.waitTime = waitTime;
        this.ascent = ascent;
        timer = new ElapsedTime();
        this.robotState = robotState;
    }

    @Override
    public void initialize() {
        ascent.engageJacks();
        ascent.engageAscent();
        robotState.setIsAscending(true);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > waitTime;
    }
//
//    @Override
//    public void end(boolean interrupted) {
//    }
}
