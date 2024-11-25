package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp
@SuppressWarnings("unused")
public class LocalizationTestOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        robotState = new RobotState();
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap, robotState);

        registerSubsystems(odometry);
    }

    @Override
    public void update() {
        telemetry.addData("Robot Pose: ", robotState.getRobotCurrentPose());
    }
}
