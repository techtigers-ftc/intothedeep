package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

public class TestPedroAutoDriveCommand extends PedroAutoDriveCommandBase{

    public TestPedroAutoDriveCommand(DriveSubsystem drive, RobotState robotState) {
        super(drive, robotState);
        setTranslationalPIDF(0.1, 0, 0.1, 0);
        setHeadingPIDF(0.1, 0.1, 0, 0);
        setDrivePIDF(0.1, 0.1, 0, 0.1, 0);
        setPathChain(follower.pathBuilder().addBezierCurve(new Point(0, 0), new Point(25, 25), new Point(50,0)).build());
    }
}
