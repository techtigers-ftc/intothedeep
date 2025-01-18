package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * An autonomous opmode on the basket side of the blue alliance
 */
@Autonomous
public class BlueBasketAutoOpMode extends BasketAutoOpMode {
    @Override
    protected boolean isBlue() {
        return true;
    }
}
