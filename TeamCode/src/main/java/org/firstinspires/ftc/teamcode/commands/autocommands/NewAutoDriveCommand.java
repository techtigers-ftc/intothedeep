package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.FilteredPIDFController;
import org.firstinspires.ftc.teamcode.pedropathing.util.PIDFController;

/**
 * A class for autonomous drive commands that use PedroPathing.
 */
public class NewAutoDriveCommand extends CommandBase {
    private static final String LOG_TAG =
            NewAutoDriveCommand.class.getSimpleName();

    public final Follower follower;
    private PathChain pathChain;
    private PIDFController translationalPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController drivePIDF;

    /**
     * Constructs a new AutoDriveCommand.
     */
    public NewAutoDriveCommand(HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);
    }

    @Override
    public void initialize() {
        pathChain = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(0, 0),
                                new Point(40, 0),
                                new Point(20, 40),
                                new Point(50, 60)
                        ))
                .setLinearHeadingInterpolation(0, Math.PI)
                .build();
        follower.followPath(pathChain, true);
    }

    @Override
    public void execute() {
        follower.update();
    }
}
