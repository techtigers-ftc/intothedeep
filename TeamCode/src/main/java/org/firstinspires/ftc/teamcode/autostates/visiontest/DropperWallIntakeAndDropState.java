package org.firstinspires.ftc.teamcode.autostates.visiontest;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.ReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;
import team.techtigers.base.statemachine.SequentialCommandGroupState;

public class DropperWallIntakeAndDropState extends SequentialCommandGroupState<AutoState> {
    private final RobotState robotState;

    public DropperWallIntakeAndDropState(String name, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, 1.5);
        this.robotState = robotState;

        addCommands(
                new DropperWallIntakeAction(dropper, intake, robotState)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getDropperState() == DropperState.WALL_INTAKE) {
            return AutoState.DRIVE_END;
        }
        return AutoState.RUNNING;
    }
}
