package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;
import team.techtigers.core.paths.Waypoint;

@TeleOp(name = "Localization Tuning OpMode", group = "Tuning")
public class LocalizationTuningOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        robotState = new RobotState();
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap, robotState);

        registerSubsystems(odometry);
    }

    @Override
    public void update() {
        Waypoint robotPose = robotState.getRobotCurrentPose();
        telemetry.addData("Robot X: ", robotPose.getX());
        telemetry.addData("Robot Y: ", robotPose.getY());
        telemetry.addData("Robot Heading: ", robotPose.getHeading());
    }
}
