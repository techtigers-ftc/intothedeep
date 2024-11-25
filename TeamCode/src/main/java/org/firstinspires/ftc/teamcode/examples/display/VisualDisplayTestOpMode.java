package org.firstinspires.ftc.teamcode.examples.display;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.visualdisplay.AdafruitNeoPixel;
import team.techtigers.base.visualdisplay.VisualDisplaySubsystem;
import team.techtigers.core.display.DisplayView;

/**
 * Test op mode for the visual display (LEDs)
 */
@TeleOp
public class VisualDisplayTestOpMode extends BaseOpMode {

    @Override
    public void initialize() {
        // Initialize display. We have 384 leds on our display, and each one
        // is RGB, hence the 3 bytes. RGBW Leds would have 4 bytes.
        AdafruitNeoPixel visualDisplay = hardwareMap.get(AdafruitNeoPixel.class, "visualDisplay");
        visualDisplay.initialize(384, 3);

        // Creates a view, a set of regions which contain sprites
        DisplayView defaultView = new ExampleView();

        // Subsystem creation
        VisualDisplaySubsystem displaySubsystem =
                new VisualDisplaySubsystem(visualDisplay, defaultView);

        // Register subsystem so it runs every cycle
        registerSubsystems(displaySubsystem);
    }
}
