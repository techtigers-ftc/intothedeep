package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * An autonomous opmode on the basket side of the red alliance
 */
@Autonomous
public class RedBasketAutoOpMode extends BasketAutoOpMode {
    @Override
    protected boolean isBlue() {
        return false;
    }
}
