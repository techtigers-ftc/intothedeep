package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp
public class ClawCamTestOpMode extends BaseOpMode {
    private RobotState robotState;
    private VisionSubsystem visionSubsystem;
    @Override
    public void initialize() {
        visionSubsystem = new VisionSubsystem(hardwareMap, robotState);
        registerSubsystems(visionSubsystem);
    }
}
