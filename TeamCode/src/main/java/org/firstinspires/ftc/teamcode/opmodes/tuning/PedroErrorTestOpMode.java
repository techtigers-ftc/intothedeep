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
        driveToPlace.setTranslationalPIDF(FollowerConstants.translationalPIDFCoefficients.P,
                FollowerConstants.translationalPIDFCoefficients.I,
                FollowerConstants.translationalPIDFCoefficients.D,
                FollowerConstants.translationalPIDFCoefficients.F);
        driveToPlace.setHeadingPIDF(FollowerConstants.headingPIDFCoefficients.P,
                FollowerConstants.headingPIDFCoefficients.I,
                FollowerConstants.headingPIDFCoefficients.D,
                FollowerConstants.headingPIDFCoefficients.F);
        driveToPlace.setDrivePIDF(FollowerConstants.drivePIDFCoefficients.P,
                FollowerConstants.drivePIDFCoefficients.I,
                FollowerConstants.drivePIDFCoefficients.D,
                FollowerConstants.drivePIDFCoefficients.T,
                FollowerConstants.drivePIDFCoefficients.F);

        driveToPlace.setSecondaryTranslationalPIDF(FollowerConstants.secondaryTranslationalPIDFCoefficients.P,
                FollowerConstants.secondaryTranslationalPIDFCoefficients.I,
                FollowerConstants.secondaryTranslationalPIDFCoefficients.D,
                FollowerConstants.secondaryTranslationalPIDFCoefficients.F);
        driveToPlace.setSecondaryHeadingPIDF(FollowerConstants.secondaryHeadingPIDFCoefficients.P,
                FollowerConstants.secondaryHeadingPIDFCoefficients.I,
                FollowerConstants.secondaryHeadingPIDFCoefficients.D,
                FollowerConstants.secondaryHeadingPIDFCoefficients.F);
        driveToPlace.setSecondaryDrivePIDF(FollowerConstants.secondaryDrivePIDFCoefficients.P,
                FollowerConstants.secondaryDrivePIDFCoefficients.I,
                FollowerConstants.secondaryDrivePIDFCoefficients.D,
                FollowerConstants.secondaryDrivePIDFCoefficients.T,
                FollowerConstants.secondaryDrivePIDFCoefficients.F);

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
