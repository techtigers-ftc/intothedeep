package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * A TeleOp mode for the blue alliance one driver
 */
@TeleOp
@SuppressWarnings("unused")
public class BlueSinglePlayerTeleOpMode extends BaseSinglePlayerTeleOpMode{
    @Override
    protected boolean isBlue() {
        return true;
    }
}
