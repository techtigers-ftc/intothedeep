package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;

/**
 * A test autonomous drive command that uses PedroPathing.
 */
public class FirstDriveToBasketCommand extends AutoDriveCommandBase {

    /**
     * Constructs a new FirstDriveToBasketCommand.
     *
     * @param drive      The drive subsystem
     * @param robotState The robot state
     */
    public FirstDriveToBasketCommand(DriveSubsystem drive, RobotState robotState) {
        super(drive, robotState);
        setTranslationalPIDF(TuningConstants.translationalP,
                TuningConstants.translationalI,
                TuningConstants.translationalD, 0);
        setHeadingPIDF(TuningConstants.headingP, TuningConstants.headingI,
                TuningConstants.headingD, 0);
        setDrivePIDF(TuningConstants.driveP, TuningConstants.driveI,
                TuningConstants.driveD, 0, 0);

        setPathChain(
                follower.pathBuilder()
                        .addPath(
                            new BezierLine(
                                    new Point(29.75, 7.25),
                                    new Point(13, 13)
                            )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90),
                                Math.toRadians(45))
                        .build()
        );
    }
}
