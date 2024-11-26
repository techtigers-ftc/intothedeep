package org.firstinspires.ftc.teamcode.opmodes.configurators;

import org.firstinspires.ftc.teamcode.commands.PedroPathFollowCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * Configurator used for testing purposes
 */
public class TestConfigurator implements iConfigurator {
    private final PathChain drivePathChain;
    PathBuilder pathBuilder;

    public TestConfigurator() {
        pathBuilder = new PathBuilder();
        pathBuilder.addBezierCurve(new Point(0, 0), new Point(12, 12), new Point(24, 0));
        pathBuilder.addBezierCurve(new Point(24, 0), new Point(36, 12), new Point(48, 0));
        drivePathChain = pathBuilder.build();
    }

    @Override
    public void configTestDrive(PedroPathFollowCommand command) {
        command.setSecondaryTranslationalPIDF(1, 0, 1);
        command.setTranslationalPIDF(1, 0, 1);

        command.setPathChain(drivePathChain);
    }
}
