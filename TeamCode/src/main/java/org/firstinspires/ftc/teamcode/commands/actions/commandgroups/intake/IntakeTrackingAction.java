package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

public class IntakeTrackingAction extends CommandBase {
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    private static final double FINAL_Y = 100;
    private double tolerance;
    public IntakeTrackingAction(IntakeSubsystem intake, double tolerance, RobotState robotState){
        this.intake = intake;
        this.tolerance = tolerance;
        this.robotState = robotState;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if(robotState.getBlockForwardFine() - FINAL_Y > 0) {
            intake.moveSlidesRelative(-0.25);
        }else{
            intake.moveSlidesRelative(0.25);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(robotState.getBlockForwardFine() - FINAL_Y) < tolerance;
    }
}
