package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * An autonomous opmode on the basket side of the blue alliance
 */
@Autonomous(name = "Blue Basket 0+6", group = "Basket Auto")
public class BlueBasketAutoOpMode extends BasketAutoOpMode {
    @Override
    protected boolean isBlue() {
        return true;
    }
}
