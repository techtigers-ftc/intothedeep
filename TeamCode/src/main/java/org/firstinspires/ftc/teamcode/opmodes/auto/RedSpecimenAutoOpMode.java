package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * An autonomous opmode on the specimen side of the red alliance, clipping 5 specimens and dropping a sample
 */
@Autonomous(name = "Red Specimen 6+0", group = "Specimen Auto")
public class RedSpecimenAutoOpMode extends SpecimenAutoOpMode {
    @Override
    protected boolean isBlue() {
        return false;
    }
}
