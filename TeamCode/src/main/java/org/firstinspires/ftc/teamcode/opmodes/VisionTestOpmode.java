package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SimpleVisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

/**
 * Test opmode for running just the LimelightSubsystem
 */
@TeleOp
@SuppressWarnings("unused")
public class VisionTestOpmode extends BaseOpMode {

    @Override
    public void initialize() {
        SimpleVisionSubsystem vision = new SimpleVisionSubsystem(hardwareMap, new RobotState());
        registerSubsystems(vision);
    }
}
