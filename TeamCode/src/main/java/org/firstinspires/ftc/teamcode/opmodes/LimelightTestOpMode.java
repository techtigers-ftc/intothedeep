package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

/**
 * Test opmode for running just the LimelightSubsystem
 */
@TeleOp
@SuppressWarnings("unused")
public class LimelightTestOpMode extends BaseOpMode {

    @Override
    public void initialize() {
        // TODO: input actual values below
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, new RobotState(), 10, 5, 0, 0);
        registerSubsystems(limelight);
    }
}
