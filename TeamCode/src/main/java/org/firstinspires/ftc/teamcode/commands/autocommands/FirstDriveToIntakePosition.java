package org.firstinspires.ftc.teamcode.commands.autocommands;

import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;

/**
 * A test autonomous drive command that uses PedroPathing.
 */
public class FirstDriveToIntakePosition extends AutoDriveCommandBase {

    /**
     * Constructs a new FirstDriveToBasketCommand.
     *
     * @param drive      The drive subsystem
     * @param robotState The robot state
     */
    public FirstDriveToIntakePosition(DriveSubsystem drive, RobotState robotState) {
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
                                    new Point(12, 12),
                                    new Point(15, 19)
                            )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(78))
                        .build()
        );
    }
}
