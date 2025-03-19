package org.firstinspires.ftc.teamcode.commands.actions.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autostates.specimen.ClipSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.PickupSpecimenState;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperForwardCarryWallAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperWallIntakeNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.drive.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.opmodes.auto.SpecimenDriveStateConfigurator;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command group that automatically drives and slaps the specimens for the teleop period
 */
public class AutoSpecimenCycleAction extends SequentialCommandGroup {
    private static final String LOG_TAG = AutoSpecimenCycleAction.class.getSimpleName();

    public AutoSpecimenCycleAction(DriveSubsystem drive, DropperSubsystem dropper, RobotState robotState) {
        addRequirements(drive, dropper);
        addCommands(
                new PickupSpecimenState("pickupSpecimen", drive, dropper, robotState),
                new ParallelCommandGroup(
                        new TeleDriveCommand(drive,
                                new CustomPIDFCoefficients(0.08, 0, 0.001, 0),
                                new CustomFilteredPIDFCoefficients(0.0055, 0, 0.0035, 0.6, 0),
                                new CustomPIDFCoefficients(0.9, 0, 0.015, 0),
                                new Pose(68, 41.5, 90),
                                robotState,
                                SpecimenDriveStateConfigurator.MEDIUM_TOLERANCE,
                                SpecimenDriveStateConfigurator.LARGE_ANGLE_TOLERANCE,
                                3
                        ),
                        new DropperForwardCarryWallAction(dropper, robotState)
                ),
                new ClipSpecimenState("clipSpecimen", dropper, drive, robotState),
                // Drive back
                new ParallelCommandGroup(
                        new TeleDriveCommand(drive,
                                new CustomPIDFCoefficients(0.03, 0, 0.001, 0),
                                new CustomFilteredPIDFCoefficients(0.008, 0, 0.0045, 0.6, 0),
                                new CustomPIDFCoefficients(0.5, 0, 0.03, 0), new Pose(113, 13, 90), robotState, SpecimenDriveStateConfigurator.LARGE_TOLERANCE, SpecimenDriveStateConfigurator.LARGE_ANGLE_TOLERANCE, 3
                        ),
                        new DropperWallIntakeNoTransferAction(dropper, robotState)
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            this.schedule();
        }
    }
}
