package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPlace;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.pedropathing.follower.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;

import team.techtigers.base.BaseOpMode;
import team.techtigers.core.paths.Waypoint;

@Config
@Autonomous
public class PedroErrorTestOpMode extends BaseOpMode {
    public static double targetX = 0;
    public static double targetY = -30;
//    public static double targetH = 90;
//    public static double startH = 90;

    public static double xScale = -100;
    public static double yScale = -100;

    private RobotState robotState;
    private AutoDriveCommand driveToPlace;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry,
                FtcDashboard.getInstance().getTelemetry());
        robotState = new RobotState(true, true);

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap,
                robotState, new Waypoint(0+xScale, 0+yScale,
                Math.toRadians(90)));
        registerSubsystems(drive, odometry);

        driveToPlace = new AutoDriveCommand(drive, robotState);
        driveToPlace.setTranslationalPIDF(
                TuningConstants.aTranslationalP, 0,
                TuningConstants.bTranslationalD, 0);
        driveToPlace.setHeadingPIDF(
                TuningConstants.eHeadingP, 0,
                TuningConstants.fHeadingD, 0);
        driveToPlace.setDrivePIDF(
                TuningConstants.cDriveP, 0,
                TuningConstants.dDriveD, 0.6, 0);

        driveToPlace.setSecondaryTranslationalPIDF(
                TuningConstants.gSecondaryTranslationalP, 0,
                TuningConstants.hSecondaryTranslationalD, 0);
        driveToPlace.setSecondaryHeadingPIDF(
                TuningConstants.kSecondaryHeadingP, 0,
                TuningConstants.lSecondaryHeadingD, 0);
        driveToPlace.setSecondaryDrivePIDF(
                TuningConstants.iSecondaryDriveP, 0,
                TuningConstants.jSecondaryDriveD, 0.6, 0);

        driveToPlace.setPathChain(new PathBuilder().addBezierLine(
                        new Point(0+xScale,0+yScale),
                        new Point(targetX+xScale, targetY+yScale)
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
//                .addBezierLine(
//                        new Point(74+xScale, 40.25+yScale),
//                        new Point(101+xScale, 33+yScale)
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build()
        );

//        driveToPlace.setTolerance(3);
//        driveToPlace.setAngleTolerance(Math.toRadians(5));

        telemetry.addData("Current X", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Target X",
                0);
        telemetry.update();
    }

    @Override
    public void justAfterStart() {
        driveToPlace.schedule();
    }

    @Override
    public void update() {
        telemetry.addData("Target T (PP)",
                driveToPlace.follower.getCurrentTValue());
        telemetry.addData("Target Pose (PP)",
                driveToPlace.follower.getClosestPose());
        telemetry.addLine();
        telemetry.addData("Current Pose (RS)",
                robotState.getRobotCurrentPose());

        telemetry.addData("Current X", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Target X",
                driveToPlace.follower.getClosestPose().getX());
        telemetry.update();
    }
}
