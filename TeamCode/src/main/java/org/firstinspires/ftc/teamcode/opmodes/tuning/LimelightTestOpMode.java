package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;
import team.techtigers.core.paths.Waypoint;

@TeleOp(name = "Limelight Test OpMode", group = "Tuning")
public class LimelightTestOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        robotState = new RobotState(true, false);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState);

        registerSubsystems(limelight);
    }

    @Override
    public void update() {
        telemetry.addData("Lateral Coarse Distance:", robotState.getBlockLateralCoarse());
        telemetry.addData("Forward Coarse Distance", robotState.getBlockForwardCoarse());
        telemetry.addData("Block Orientation", robotState.getBlockOrientation());
    }
}
