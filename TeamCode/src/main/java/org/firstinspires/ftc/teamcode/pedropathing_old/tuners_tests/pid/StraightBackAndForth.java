package org.firstinspires.ftc.teamcode.pedropathing_old.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.pedropathing_old.localization.localizers.RobotStateLocalizer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing_old.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing_old.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing_old.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;


/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Disabled
@Autonomous (name = "Straight Back And Forth ", group = "PIDF Tuning")
public class StraightBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private Path forwards;
    private Path backwards;

    private GoBodometrySubsystem odometry;
    private DriveSubsystem drive;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        RobotState robotState = new RobotState(true, true);
        RobotStateLocalizer robotStateLocalizer = new RobotStateLocalizer(robotState);
        odometry = new GoBodometrySubsystem(hardwareMap, robotState);
        drive = new DriveSubsystem(hardwareMap, robotState);
        follower = new Follower(robotStateLocalizer);

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);

        follower.followPath(forwards);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                            + " inches forward. The robot will go forward and backward continuously"
                            + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        odometry.periodic();
        drive.periodic();
        follower.update();
        drive.drivePedroPath(follower.getCurrentDriveVectors());

        follower.setTranslationalPIDF(new CustomPIDFCoefficients(TuningConstants.aTranslationalP,0,TuningConstants.bTranslationalD,0));
        follower.setHeadingPIDF(new CustomPIDFCoefficients(TuningConstants.eHeadingP,0,TuningConstants.fHeadingD,0));
        follower.setDrivePIDF(new CustomFilteredPIDFCoefficients(TuningConstants.cDriveP,0,TuningConstants.dDriveD,0.6,0));
//        follower.setSecondaryTranslationalPIDF(new CustomPIDFCoefficients(TuningConstants.gSecondaryTranslationalP,0,TuningConstants.hSecondaryTranslationalD,0));
//        follower.setSecondaryHeadingPIDF(new CustomPIDFCoefficients(TuningConstants.kSecondaryHeadingP,0,TuningConstants.lSecondaryHeadingD,0));
//        follower.setSecondaryDrivePIDF(new FilteredPIDFController(new CustomFilteredPIDFCoefficients(TuningConstants.iSecondaryDriveP,0,TuningConstants.jSecondaryDriveD,0.6,0));

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
