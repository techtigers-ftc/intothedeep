package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.TestPedroAutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@Autonomous(name = "Pedro Test AutoDrive OpMode", group = "Autonomous")
public class PedroTestAutoDriveOpMode extends BaseOpMode {
    private RobotState robotState;
    @Override
    public void initialize() {
        robotState = new RobotState();

        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap, robotState);
        registerSubsystems(drive, odometry);

        TestPedroAutoDriveCommand command = new TestPedroAutoDriveCommand(drive, robotState);
        CommandScheduler.getInstance().schedule(command);
    }

    @Override
    public void update() {
        telemetry.addData("pose", robotState.getRobotCurrentPose());
    }
}
