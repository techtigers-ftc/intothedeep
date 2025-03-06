package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * An autonomous opmode on the specimen side of the red alliance, clipping 5 specimens and dropping a sample
 */
@Autonomous(name = "Red Specimen 5+1", group = "Specimen Auto")
public class RedSpecimenSampleAutoOpMode extends SpecimenAutoOpMode {
    @Override
    protected boolean isBlue() {
        return false;
    }

    @Override
    protected boolean doSample() {
        return true;
    }
}
