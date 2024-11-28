package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.TestPedroAutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

public class PedroTestAutoDriveOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        RobotState robotState = new RobotState();
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        registerSubsystems(drive);

        TestPedroAutoDriveCommand command = new TestPedroAutoDriveCommand(drive, robotState);
        CommandScheduler.getInstance().schedule(command);
    }
}
