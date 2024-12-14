package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

/**
 * Test opmode for running just the LimelightSubsystem
 */
@TeleOp
@Config
@SuppressWarnings("unused")
public class LimelightTestOpMode extends BaseOpMode {
    public static double yOffset = 4.5;
    public static double yHeight = 10.75;
    @Override
    public void initialize() {
        // TODO: input actual values below
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, new RobotState(), yHeight, 4.9, yOffset,25);
        registerSubsystems(limelight);
    }
}
