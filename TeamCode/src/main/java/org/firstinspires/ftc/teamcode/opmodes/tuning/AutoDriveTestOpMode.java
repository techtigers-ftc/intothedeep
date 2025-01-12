package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autostates.TestAutoDriveState;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.statemachine.StateMachine;
import team.techtigers.core.paths.Waypoint;

@Autonomous(name = "Auto Test Drive OpMode")
public class AutoDriveTestOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        StateMachine<AutoState> stateMachine = new StateMachine<>();
        robotState = new RobotState(true, true);

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap, robotState, new Waypoint(29.75, 7.25, Math.toRadians(90)));

        TestAutoDriveState testAutoDrive = new TestAutoDriveState("testAutoDrive", drive, robotState);
        testAutoDrive.setPIDSToFollowerConstants();
        testAutoDrive.setTranslationalPIDF(0.1,0,0.01,0);
        testAutoDrive.setSecondaryTranslationalPIDF(0.15,0,0.01,0);
        testAutoDrive.setHeadingPIDF(1,0,0.03,0);
        testAutoDrive.setSecondaryHeadingPIDF(1,0,0.06,0);
        testAutoDrive.setDrivePIDF(0.002,0,0.00035,0.6,0);
        testAutoDrive.setSecondaryDrivePIDF(0.003,0,0.0002,0.6,0);
        testAutoDrive.setTolerance(1);
        testAutoDrive.setAngleTolerance(Math.toRadians(1));
        testAutoDrive
                .setPathChain(
                        new PathBuilder().addBezierLine(
                                                new Point(29.75, 7.25),
                                                new Point(12, 12)
                                        )
                                .setLinearHeadingInterpolation(Math.toRadians(90),
                                        Math.toRadians(45))
                                .build()
                );

        TestAutoDriveState secondTestAutoDrive = new TestAutoDriveState("secondTestAutoDrive", drive, robotState);
        secondTestAutoDrive.setPIDSToFollowerConstants();
        secondTestAutoDrive.setTolerance(1);
        secondTestAutoDrive.setAngleTolerance(Math.toRadians(1));
        secondTestAutoDrive
                .setPathChain(new PathBuilder().addBezierCurve(new Point(15, 15), new Point(0, 0))
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                        .build());

        stateMachine
                .addState(testAutoDrive)
                .addState(secondTestAutoDrive)

//                .addTransition(testAutoDrive, secondTestAutoDrive, AutoState.DRIVE_END)
                .setCurrentState(testAutoDrive);
        ;

        AutoSubsystem auto = new AutoSubsystem(stateMachine);
        registerSubsystems(auto, drive, odometry);

        update();
        telemetry.update();
    }

    @Override
    public void update() {
        telemetry.addData("Current Pose", robotState.getRobotCurrentPose());
        telemetry.addData("Final Pose", robotState.getRobotFinalPose());
        telemetry.addLine();
        telemetry.addData("Current X", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Current Y", robotState.getRobotCurrentPose().getY());
        telemetry.addData("Current Heading", Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Expected X", robotState.getRobotFinalPose().getX());
        telemetry.addData("Expected Y", robotState.getRobotFinalPose().getY());
        telemetry.addData("Expected Heading", robotState.getRobotFinalPose().getHeading());
    }
}
