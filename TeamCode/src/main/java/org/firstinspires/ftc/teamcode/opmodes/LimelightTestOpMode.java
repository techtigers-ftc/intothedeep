package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.GlobalConstants;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

/**
 * Test opmode for running just the LimelightSubsystem
 */
@TeleOp
@Config
@SuppressWarnings("unused")
public class LimelightTestOpMode extends BaseOpMode {
    public static double Y_OFFSET = 5.5;
    public static double Y_HEIGHT = 10.5;
    public static double X_OFFSET = 4.9;
    public static double DOWNWARDS_ANGLE = 25;
    private RobotState robotState;
    @Override
    public void initialize() {
        robotState = new RobotState();
        GlobalConstants.initialize(false, true);

        // TODO: input actual values below
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState, Y_HEIGHT, X_OFFSET, Y_OFFSET, DOWNWARDS_ANGLE);
        registerSubsystems(limelight);
    }

    @Override
    public void update() {
        telemetry.addData("Lateral Sample Coarse Distance", robotState.getBlockLateralCoarse());
        telemetry.addData("Forward Sample Coarse Distance", robotState.getBlockForwardCoarse());
    }
}
