package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * A TeleOp mode for the red alliance one driver
 */
@TeleOp
@SuppressWarnings("unused")
public class RedSinglePlayerTeleOpMode extends BaseSinglePlayerTeleOpMode{
    @Override
    protected boolean isBlue() {
        return false;
    }
}
