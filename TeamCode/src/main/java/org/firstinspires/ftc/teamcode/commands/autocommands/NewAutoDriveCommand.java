package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.FilteredPIDFController;
import org.firstinspires.ftc.teamcode.pedropathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseTranslator;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.paths.Waypoint;

/**
 * A class for autonomous drive commands that use PedroPathing.
 */
public class NewAutoDriveCommand extends CommandBase {
    private static final String LOG_TAG =
            NewAutoDriveCommand.class.getSimpleName();

    private final Follower follower;
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
        follower.followPath(pathChain, true);
    }

    @Override
    public void execute() {
        follower.update();
    }
}
