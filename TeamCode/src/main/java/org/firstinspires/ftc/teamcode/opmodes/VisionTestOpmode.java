package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;

import team.techtigers.base.BaseOpMode;

/**
 * Test opmode for running just the VisionSubsystem
 */
@TeleOp
@SuppressWarnings("unused")
public class VisionTestOpmode extends BaseOpMode {
    RobotState robotState;

    @Override
    public void initialize() {
        robotState = new RobotState(true, false);
        robotState.setBlockColorPreference(BlockColorPreference.ALLIANCE);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap, robotState);
        registerSubsystems(vision);
    }

    @Override
    public void update() {
        telemetry.addData("Forward fine", robotState.getBlockForwardFine());
        telemetry.addData("Lateral fine", robotState.getBlockLateralFine());
        telemetry.addData("Orientation", robotState.getBlockOrientation());
    }
}
