package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * A TeleOp mode for the red alliance
 */
@TeleOp(name = "Red TeleOp", group = "Double Player TeleOp")
@SuppressWarnings("unused")
public class RedTeleOpMode extends BaseTeleOpMode{
    @Override
    protected boolean isBlue() {
        return false;
    }
}
