package org.firstinspires.ftc.teamcode.examples.display;

import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.DisplayView;

/**
 * An example view period
 */
public class ExampleView extends DisplayView {

    /**
     * Creates a new view
     */
    public ExampleView() {
        super(new DisplayRegion[]{
                new ExampleRegion(0, 0)
        });
    }
}
