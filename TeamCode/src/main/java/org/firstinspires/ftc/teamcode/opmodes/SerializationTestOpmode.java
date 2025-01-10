package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.base.BaseOpMode;

@TeleOp
public class SerializationTestOpmode extends BaseOpMode {

    @Override
    public void initialize() {
        robotState = new RobotState();
    }
}
