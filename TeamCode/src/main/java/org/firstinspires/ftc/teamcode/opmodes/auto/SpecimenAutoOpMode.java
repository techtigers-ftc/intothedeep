package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstIntake;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropStateSpecimen;
import org.firstinspires.ftc.teamcode.autostates.specimen.ClipSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.EndState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPlace;
import org.firstinspires.ftc.teamcode.autostates.specimen.GrabFirstSampleState;
import org.firstinspires.ftc.teamcode.autostates.specimen.GrabSampleState;
import org.firstinspires.ftc.teamcode.autostates.specimen.OpenIntakeState;
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
                distToIntakeTarget(robotState, new Waypoint(119.5, 45)),
                () -> 45
        );

        DriveToPlace driveToFirstDropoff = new DriveToPlace(
                "driveToFirstDropoff",
                drive,
                robotState);
        SpecimenDriveStateConfigurator.configFirstDrop(driveToFirstDropoff);


        OpenIntakeState dropFirstSample = new OpenIntakeState(
                "dropFirstSample",
                robotState,
                intake,
                dropper
        );

        DriveToPlace driveToSecondIntake = new DriveToPlace(
                "driveToSecondIntake",
                drive,
                robotState);
        SpecimenDriveStateConfigurator.configSecondIntake(driveToSecondIntake);

        GrabSampleState grabSecondSample = new GrabSampleState(
                "grabSecondSample",
                intake,
                dropper,
                robotState
        );

        DriveToPlace driveToSecondDropoff = new DriveToPlace(
                "driveToSecondDropoff",
                drive,
                robotState);
        SpecimenDriveStateConfigurator.configSecondDrop(driveToSecondDropoff);

        OpenIntakeState dropSecondSample = new OpenIntakeState(
                "dropSecondSample",
                robotState,
                intake,
                dropper
        );

        DriveToPlace driveToThirdIntake = new DriveToPlace(
                "driveToThirdIntake",
                drive,
                robotState);
        SpecimenDriveStateConfigurator.configThirdIntake(driveToThirdIntake);

        GrabSampleState grabThirdSample = new GrabSampleState(
                "grabThirdSample",
                intake,
                dropper,
                robotState
        );

        DriveToPlace driveToThirdDropoff = new DriveToPlace(
                "driveToThirdDropoff",
                drive,
                robotState);
        SpecimenDriveStateConfigurator.configThirdDrop(driveToThirdDropoff);

        OpenIntakeState dropThirdSample = new OpenIntakeState(
                "dropThirdSample",
                robotState,
                intake,
                dropper
        );
        
        EndState endState = new EndState("end");

        // Create the state machine
        stateMachine
                .addState(driveChamberPreload)
                .addState(clipSpecimen)
                .addState(driveToFirstIntake)
                .addState(grabFirstSample)
                .addState(driveToFirstDropoff)
                .addState(dropFirstSample)
                .addState(driveToSecondIntake)
                .addState(grabSecondSample)
                .addState(driveToSecondDropoff)
                .addState(dropSecondSample)
                .addState(driveToThirdIntake)
                .addState(grabThirdSample)
                .addState(driveToThirdDropoff)
                .addState(dropThirdSample)
                .addState(endState)

                .addTransition(driveChamberPreload, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(clipSpecimen, driveToFirstIntake, AutoState.SPECIMEN_1_DROP_COMPLETE)
                .addTransition(driveToFirstIntake, grabFirstSample, AutoState.DRIVE_END)
                .addTransition(grabFirstSample, driveToFirstDropoff, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(driveToFirstDropoff, dropFirstSample, AutoState.DRIVE_END)
                .addTransition(dropFirstSample, driveToSecondIntake, AutoState.SPECIMEN_1_DROP_COMPLETE)
                .addTransition(driveToSecondIntake, grabSecondSample, AutoState.DRIVE_END)
                .addTransition(grabSecondSample, driveToSecondDropoff, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(driveToSecondDropoff, dropSecondSample,AutoState.DRIVE_END)
                .addTransition(dropSecondSample, driveToThirdIntake, AutoState.SPECIMEN_1_DROP_COMPLETE)
                .addTransition(driveToThirdIntake, grabThirdSample, AutoState.DRIVE_END)
                .addTransition(grabThirdSample, driveToThirdDropoff, AutoState.SAMPLE_1_DROP_COMPLETE)
                .addTransition(driveToThirdDropoff, dropSecondSample, AutoState.DRIVE_END)


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
