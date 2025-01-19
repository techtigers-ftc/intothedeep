package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autostates.specimen.ClipSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstIntake;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropStateSpecimen;
import org.firstinspires.ftc.teamcode.autostates.specimen.DropSampleState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DropSecondAndThirdSampleState;
import org.firstinspires.ftc.teamcode.autostates.specimen.EndState;
import org.firstinspires.ftc.teamcode.autostates.specimen.GrabFirstSampleState;
import org.firstinspires.ftc.teamcode.autostates.specimen.GrabOtherSampleState;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import java.util.function.DoubleSupplier;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.statemachine.StateMachine;
import team.techtigers.core.paths.Waypoint;

@Autonomous
public class SpecimenAutoOpMode extends BaseOpMode {
    private RobotState robotState;
    private IntakeSubsystem intake;

    private DoubleSupplier distToIntakeTarget(RobotState robotState, Waypoint target) {
        return () -> Math.min(Math.hypot(target.getX() - robotState.getRobotCurrentPose().getX(),
                target.getY() - robotState.getRobotCurrentPose().getY()) - 9, 19);
    }

    @Override
    public void initialize() {
        StateMachine<AutoState> stateMachine = new StateMachine<>();
        robotState = new RobotState(true, true);

        // Initialize subsystems
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap,
                robotState, new Waypoint(77, 7.25, Math.toRadians(90)));
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap,
                robotState);
        intake = new IntakeSubsystem(hardwareMap, robotState);

        // Creating states
        DriveToPreloadDropStateSpecimen driveChamberPreload = new DriveToPreloadDropStateSpecimen(
                "driveToChamberPreload",
                drive,
                dropper,
                robotState);
        SpecimenDriveStateConfigurator.configPreloadDrop(driveChamberPreload);

        ClipSpecimenState clipSpecimen = new ClipSpecimenState(
                "clipSpecimen",
                dropper,
                robotState);

        DriveToFirstIntake driveToFirstIntake = new DriveToFirstIntake(
                "driveToFirstIntake",
                drive,
                robotState
        );
        SpecimenDriveStateConfigurator.configFirstIntake(driveToFirstIntake);

        GrabFirstSampleState grabFirstSample = new GrabFirstSampleState(
                "grabFirstSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(119.5, 46)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading())
        );

        DropSampleState dropFirstSample = new DropSampleState(
                "dropFirstSample",
                intake,
                dropper,
                robotState,
                drive
        );
        SpecimenDriveStateConfigurator.configureFirstHoldPoint(dropFirstSample);

        GrabOtherSampleState grabSecondSample = new GrabOtherSampleState(
                "grabSecondSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(129.5, 46)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading())
        );

        DropSecondAndThirdSampleState dropSecondSample = new DropSecondAndThirdSampleState(
                "dropSecondSample",
                intake,
                dropper,
                robotState,
                drive,
                distToIntakeTarget(robotState, new Waypoint(139.5, 46))
        );
        SpecimenDriveStateConfigurator.configureSecondHoldPoint(dropSecondSample);

        GrabOtherSampleState grabThirdSample = new GrabOtherSampleState(
                "grabThirdSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(139.5, 46)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading())
        );

        DropSecondAndThirdSampleState dropThirdSample = new DropSecondAndThirdSampleState(
                "dropThirdSample",
                intake,
                dropper,
                robotState,
                drive,
                () -> 0
        );
        SpecimenDriveStateConfigurator.configureSecondHoldPoint(dropSecondSample);


        EndState endState = new EndState("end");

        // Create the state machine
        stateMachine
                .addState(driveChamberPreload)
                .addState(clipSpecimen)
                .addState(driveToFirstIntake)
                .addState(grabFirstSample)
                .addState(dropFirstSample)
                .addState(grabSecondSample)
                .addState(dropSecondSample)
                .addState(grabThirdSample)
                .addState(dropThirdSample)
                .addState(endState)

                .addTransition(driveChamberPreload, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(clipSpecimen, driveToFirstIntake, AutoState.SPECIMEN_1_DROP_COMPLETE)
                .addTransition(driveToFirstIntake, grabFirstSample, AutoState.DRIVE_END)
                .addTransition(grabFirstSample, dropFirstSample, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(dropFirstSample, grabSecondSample, AutoState.SAMPLE_0_DROP_COMPLETE)
                .addTransition(grabSecondSample, dropSecondSample, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(dropSecondSample, grabThirdSample, AutoState.SAMPLE_0_DROP_COMPLETE)
                .addTransition(grabThirdSample, dropThirdSample, AutoState.SAMPLE_INTAKE_COMPLETE)


                .setCurrentState(driveChamberPreload);


        // Register subsystems + Create state machine subsystem
        AutoSubsystem auto = new AutoSubsystem(stateMachine);
        registerSubsystems(auto, drive, odometry, dropper, intake);
    }

    @Override
    public void update() {
        telemetry.addData("Current X", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Current Y", robotState.getRobotCurrentPose().getY());
        telemetry.addData("Current Heading", Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Expected X", robotState.getRobotFinalPose().getX());
        telemetry.addData("Expected Y", robotState.getRobotFinalPose().getY());
        telemetry.addData("Expected Heading", Math.toDegrees(robotState.getRobotFinalPose().getHeading()));
        telemetry.addData("Slide Position", intake.getCurrentSlidePositionInches());
    }
}
