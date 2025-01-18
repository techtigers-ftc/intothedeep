package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * A TeleOp mode for the blue alliance
 */
@TeleOp
@SuppressWarnings("unused")
public class BlueTeleOpMode extends BaseTeleOpMode{
    @Override
    protected boolean isBlue() {
        return true;
    }
}
