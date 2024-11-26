package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.PedroPathFollowCommand;
import org.firstinspires.ftc.teamcode.opmodes.configurators.TestConfigurator;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name = "Testing Pedro Pathing", group = "Autonomous")
public class TestPedroDriveOpMode extends ConfigOpModeAuto {
    private final Pose startPose = new Pose(7, 7);

    private final Point abortPoint = new Point(144 - 83.5, 120, Point.CARTESIAN);
    PedroPathFollowCommand testDriveCommand;

    @Override
    public void initialize() {
        super.initialize();
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        testDriveCommand = new PedroPathFollowCommand(robotState, drive, follower);
        registerSubsystems(drive);

        TestConfigurator configurator = new TestConfigurator();
        configurator.configTestDrive(testDriveCommand);
    }

    @Override
    public void justAfterStart() {
        CommandScheduler.getInstance().schedule(testDriveCommand);
    }

    @Override
    public void update() {
        super.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void end() {
    }
}


  