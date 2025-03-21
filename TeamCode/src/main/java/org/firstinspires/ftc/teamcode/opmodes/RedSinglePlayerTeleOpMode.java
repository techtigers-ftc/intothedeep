package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * A TeleOp mode for the red alliance one driver
 */
@TeleOp(name = "Red Single Player TeleOp", group = "Single Player TeleOp")
@SuppressWarnings("unused")
public class RedSinglePlayerTeleOpMode extends BaseSinglePlayerTeleOpMode{
    @Override
    protected boolean isBlue() {
        return false;
    }
}
