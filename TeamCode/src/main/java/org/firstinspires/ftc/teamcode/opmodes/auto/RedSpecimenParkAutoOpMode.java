package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * An autonomous opmode on the specimen side of the red alliance, clipping 5 specimens and parking
 */
@Autonomous(name = "Red Specimen 5+0", group = "Specimen Auto")
public class RedSpecimenParkAutoOpMode extends SpecimenAutoOpMode {
    @Override
    protected boolean isBlue() {
        return false;
    }

    @Override
    protected boolean doSample() {
        return false;
    }
}
