package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * An autonomous opmode on the specimen side of the blue alliance, clipping 5 specimens and dropping a sample
 */
@Autonomous(name = "Blue Specimen 5+1 Park", group = "Specimen Auto")
public class BlueSpecimenAutoOpMode extends SpecimenAutoOpMode {
    @Override
    protected boolean isBlue() {
        return true;
    }
}
