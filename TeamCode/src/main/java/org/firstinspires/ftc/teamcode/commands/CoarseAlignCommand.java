package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseTranslator;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.TuningConstants;

public class CoarseAlignCommand extends CommandBase {
    private final double tolerance;
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final RobotState robotState;
    private final Follower follower;

    public CoarseAlignCommand(DriveSubsystem drive, IntakeSubsystem intake, RobotState robotState, double tolerance) {
        this.drive = drive;
        this.intake = intake;
        this.robotState = robotState;
        this.tolerance = tolerance;
        follower = new Follower(robotState);
        addRequirements(drive, intake);
    }

    @Override
    public void initialize() {
        // Set the PIDF coefficients
        follower.setTranslationalPIDF(TuningConstants.translationalP, TuningConstants.translationalI, TuningConstants.translationalD, 0);
        follower.setHeadingPIDF(TuningConstants.headingP, TuningConstants.headingI, TuningConstants.headingD, 0);
        follower.setDrivePIDF(TuningConstants.driveP, TuningConstants.driveI, TuningConstants.driveD, 0, 0);

        intake.moveSlidesAbsolute(robotState.getBlockForwardCoarse());

        Pose currentPose = PoseTranslator.waypointToPose(robotState.getRobotCurrentPose());
        follower.holdPoint(new Point(currentPose.getX() + robotState.getBlockLateralCoarse(), currentPose.getY()), currentPose.getHeading());
    }

    @Override
    public void execute() {
        drive.drivePedroPath(follower.getCurrentDriveVectors());
    }
}
