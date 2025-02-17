package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;

import team.techtigers.base.BaseOpMode;


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
@Autonomous(name = "Straight Back And Forth", group = "PIDF Tuning")
public class StraightBackAndForthTuningOpMode extends BaseOpMode {
    public static double DISTANCE = 60;
    public static boolean useSecondaryPIDs = false;
    private Telemetry telemetryA;
    private boolean forward = true;
    private GoBodometrySubsystem odometry;
    private DriveSubsystem drive;
    private IntakeSubsystem intake;

    private AutoDriveCommand forwardCommand;
    private AutoDriveCommand backwardCommand;

    @Override
    public void initialize() {
        RobotState robotState = new RobotState(true, true);
        odometry = new GoBodometrySubsystem(hardwareMap, robotState);
        drive = new DriveSubsystem(hardwareMap, robotState);
        intake = new IntakeSubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);
        registerSubsystems(odometry, drive, sensor);

        forwardCommand = new AutoDriveCommand(drive,
                robotState);
        forwardCommand.setPathChain(new PathBuilder().addBezierLine(
                        new Point(0, 0,
                                Point.CARTESIAN), new Point(DISTANCE, 0,
                                Point.CARTESIAN))
                .build()
        );

        backwardCommand = new AutoDriveCommand(drive,
                robotState);
        backwardCommand.setPathChain(new PathBuilder().addBezierLine(
                        new Point(DISTANCE, 0,
                                Point.CARTESIAN), new Point(0, 0,
                                Point.CARTESIAN))
//                        .setReversed(true)
                .build()
        );

        setPids(forwardCommand);
        forwardCommand.schedule();

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
    public void update() {
        if ((forward && !forwardCommand.follower.isBusy()) || (!forward && !backwardCommand.follower.isBusy())) {
            if (forward) {
                forward = false;
                forwardCommand.cancel();
                setPids(backwardCommand);
                backwardCommand.schedule();
            } else {
                forward = true;
                backwardCommand.cancel();
                setPids(forwardCommand);
                forwardCommand.schedule();
            }
        }

        telemetryA.addData("going forward", forward);
    }

    private void setPids(AutoDriveCommand command) {
        command.setTranslationalPIDF(TuningConstants.aTranslationalP, 0, TuningConstants.bTranslationalD, 0);
        command.setHeadingPIDF(TuningConstants.eHeadingP, 0, TuningConstants.fHeadingD, 0);
        command.setDrivePIDF(TuningConstants.cDriveP, 0, TuningConstants.dDriveD, 0.6, 0);

        if (useSecondaryPIDs) {
            command.setSecondaryTranslationalPIDF(TuningConstants.gSecondaryTranslationalP, 0, TuningConstants.hSecondaryTranslationalD, 0);
            command.setSecondaryHeadingPIDF(TuningConstants.kSecondaryHeadingP, 0, TuningConstants.lSecondaryHeadingD, 0);
            command.setSecondaryDrivePIDF(TuningConstants.iSecondaryDriveP, 0, TuningConstants.jSecondaryDriveD, 0.6, 0);
        } else {
            command.follower.disableSecondaryPIDS();
        }
    }
}
