package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autostates.specimen.ClipSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.PickupSpecimenState;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.drive.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.opmodes.auto.SpecimenDriveStateConfigurator;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

import team.techtigers.core.paths.Waypoint;

/**
 * A command group that automatically drives and slaps the specimens for the teleop period.
 * The command must be started with the robot in the position where you want the wall intake to occur
 */
public class AutoSpecimenCycleAction extends SequentialCommandGroup {
    private static final String LOG_TAG = AutoSpecimenCycleAction.class.getSimpleName();
    private static final double X_TO_SLAP = 42;
    private static final double Y_TO_SLAP = 36;
    private static final double Y_TO_INTAKE = 2;
    private final RobotState robotState;
    private boolean needsReset;
    private Waypoint startPosition;

    /**
     * Creates a new AutoSpecimenCycleAction
     *
     * @param drive      the drive subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public AutoSpecimenCycleAction(DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        addRequirements(drive, dropper);
        this.robotState = robotState;
        startPosition = new Waypoint(robotState.getRobotCurrentPose().getX(),
                robotState.getRobotCurrentPose().getY(),
                robotState.getRobotCurrentPose().getHeading());
        needsReset = true;
        addCommands(
                new PickupSpecimenState("pickupSpecimen", drive, dropper, robotState),
                new ParallelCommandGroup(
                        new TeleDriveCommand(drive,
                                new CustomPIDFCoefficients(0.08, 0, 0.001, 0),
                                new CustomFilteredPIDFCoefficients(0.0055, 0, 0.0035, 0.6, 0),
                                new CustomPIDFCoefficients(0.9, 0, 0.015, 0),
                                () -> startPosition.getX() - X_TO_SLAP,
                                () -> startPosition.getY() + Y_TO_SLAP,
                                () -> startPosition.getHeading(),
                                robotState,
                                SpecimenDriveStateConfigurator.MEDIUM_TOLERANCE,
                                SpecimenDriveStateConfigurator.LARGE_ANGLE_TOLERANCE,
                                2
                        ),
                        new DropperForwardCarryNoTransferAction(dropper, robotState)
                ),
                new ClipSpecimenState("clipSpecimen", dropper, drive, robotState),
                // Drive back
                new ParallelCommandGroup(
                        new TeleDriveCommand(drive,
                                new CustomPIDFCoefficients(0.03, 0, 0.001, 0),
                                new CustomFilteredPIDFCoefficients(0.008, 0, 0.0045, 0.6, 0),
                                new CustomPIDFCoefficients(0.5, 0, 0.03, 0),
                                () -> startPosition.getX(),
                                () -> startPosition.getY() + Y_TO_INTAKE,
                                () -> startPosition.getHeading(),
                                robotState,
                                SpecimenDriveStateConfigurator.LARGE_TOLERANCE,
                                SpecimenDriveStateConfigurator.LARGE_ANGLE_TOLERANCE,
                                2
                        ),
                        new DropperWallIntakeNoTransferAction(dropper, robotState)
                )
        );
    }

    @Override
    public void initialize() {
        super.initialize();
//        RobotLog.dd(LOG_TAG,"Auto Specimen Cycle initialized, needs reset: %s", String.valueOf(needsReset));
        if (needsReset) {
            startPosition = new Waypoint(robotState.getRobotCurrentPose().getX(),
                    robotState.getRobotCurrentPose().getY(),
                    robotState.getRobotCurrentPose().getHeading());
//            RobotLog.dd(LOG_TAG, "Start position reset: %s", startPosition);
            needsReset = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted) {
            // If the command is not interrupted, recursively schedule it to run again
            this.schedule();
//            RobotLog.dd(LOG_TAG, "command scheduled again");
        } else {
            // If the command is interrupted, tell the command it needs to reset the next time you run it
            needsReset = true;
//            RobotLog.dd(LOG_TAG, "Auto Specimen Cycle interrupted");
        }
    }
}
