package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;

/**
 * A test autonomous drive command that uses PedroPathing.
 */
public class TestPedroAutoDriveCommand extends PedroAutoDriveCommandBase {

    /**
     * Constructs a new TestPedroAutoDriveCommand.
     * @param drive The drive subsystem
     * @param robotState The robot state
     */
    public TestPedroAutoDriveCommand(DriveSubsystem drive, RobotState robotState) {
        super(drive, robotState);
//        setTranslationalPIDF(1, 0, 0.1, 0);
//        setHeadingPIDF(1, 0.1, 0, 0);
//        setDrivePIDF(1, 0.1, 0, 0.1, 0);
        setTranslationalPIDF(TuningConstants.translationalP, TuningConstants.translationalI, TuningConstants.translationalD, 0);
        setHeadingPIDF(TuningConstants.headingP, TuningConstants.headingI, TuningConstants.headingD, 0);
        setDrivePIDF(TuningConstants.driveP, TuningConstants.driveI, TuningConstants.driveD, 0 ,0);
        setPathChain(follower.pathBuilder().addBezierLine(new Point(0, 0), new Point(30,-40)).setLinearHeadingInterpolation(0,-Math.PI/2).build());
    }
}
