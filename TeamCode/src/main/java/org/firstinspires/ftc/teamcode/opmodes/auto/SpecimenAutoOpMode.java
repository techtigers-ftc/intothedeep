package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autostates.specimen.ClipPreloadState;
import org.firstinspires.ftc.teamcode.autostates.specimen.ClipSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenDropState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPark;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPoseState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DropSampleState;
import org.firstinspires.ftc.teamcode.autostates.specimen.EndState;
import org.firstinspires.ftc.teamcode.autostates.specimen.GrabSampleState;
import org.firstinspires.ftc.teamcode.autostates.specimen.IntakeSpecimenHoldPointState;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
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
        FtcDashboard dashboard = FtcDashboard.getInstance();
        StateMachine<AutoState> stateMachine = new StateMachine<>();
        robotState = new RobotState(true, true);
        telemetry = dashboard.getTelemetry();

        // Initialize subsystems
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap,
                robotState, new Waypoint(77, 7.25, Math.toRadians(90)));
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap,
                robotState);
        intake = new IntakeSubsystem(hardwareMap, robotState);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState);

        // Creating states
        DriveToPreloadDropSpecimenState driveChamberPreload = new DriveToPreloadDropSpecimenState(
                "driveToChamberPreload",
                drive,
                dropper,
                robotState);
        SpecimenDriveStateConfigurator.configPreloadDrop(driveChamberPreload);

        ClipPreloadState clipPreload = new ClipPreloadState(
                "clipPreload",
                dropper,
                robotState);

        DriveToPoseState driveToFirstIntake = new DriveToPoseState(
                "driveToFirstIntake",
                drive,
                robotState
        );
        SpecimenDriveStateConfigurator.configFirstIntake(driveToFirstIntake);

        GrabSampleState grabFirstSample = new GrabSampleState(
                "grabFirstSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(119.5, 45.5)),
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

        GrabSampleState grabSecondSample = new GrabSampleState(
                "grabSecondSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(129.5, 45.5)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading())
        );

        DropSampleState dropSecondSample = new DropSampleState(
                "dropSecondSample",
                intake,
                dropper,
                robotState,
                drive
        );
        SpecimenDriveStateConfigurator.configureSecondHoldPoint(dropSecondSample);

        GrabSampleState grabThirdSample = new GrabSampleState(
                "grabThirdSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(139.5, 45.5)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading())
        );

        DropSampleState dropThirdSample = new DropSampleState(
                "dropThirdSample",
                intake,
                dropper,
                robotState,
                drive
        );
        SpecimenDriveStateConfigurator.configureThirdHoldPoint(dropThirdSample);

        IntakeSpecimenHoldPointState intakeSpecimen = new IntakeSpecimenHoldPointState(
                "intakeSpecimen",
                intake,
                dropper,
                drive,
                distToIntakeTarget(robotState, new Waypoint(111.5, 6)),
                robotState
        );
        SpecimenDriveStateConfigurator.configSpecimenIntakeHoldPoint(intakeSpecimen);

        DriveToGeneralSpecimenIntakeState driveToFirstSpecimenIntake = new DriveToGeneralSpecimenIntakeState(
                "driveToFirstSpecimenIntake",
                drive,
                dropper,
                robotState);
        SpecimenDriveStateConfigurator.configFirstSpecimenIntake(driveToFirstSpecimenIntake);

        ClipSpecimenState clipSpecimen = new ClipSpecimenState(
                "clipSpecimen",
                dropper,
                robotState
        );

        DriveToGeneralSpecimenDropState driveToFirstSpecimenDrop = new DriveToGeneralSpecimenDropState(
                "driveToFirstSpecimenDrop",
                drive,
                dropper,
                intake,
                robotState
        );
        SpecimenDriveStateConfigurator.configFirstSpecimenDrop(driveToFirstSpecimenDrop);

        DriveToGeneralSpecimenIntakeState driveToSecondSpecimenIntake = new DriveToGeneralSpecimenIntakeState(
                "driveToSecondSpecimenIntake",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configSecondSpecimenIntake(driveToSecondSpecimenIntake);

        DriveToGeneralSpecimenDropState driveToSecondSpecimenDrop = new DriveToGeneralSpecimenDropState(
                "driveToSecondSpecimenDrop",
                drive,
                dropper,
                intake,
                robotState
        );
        SpecimenDriveStateConfigurator.configSecondSpecimenDrop(driveToSecondSpecimenDrop);

        DriveToGeneralSpecimenIntakeState driveToThirdSpecimenIntake = new DriveToGeneralSpecimenIntakeState(
                "driveToThirdSpecimenIntake",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configThirdSpecimenIntake(driveToThirdSpecimenIntake);

        DriveToGeneralSpecimenDropState driveToThirdSpecimenDrop = new DriveToGeneralSpecimenDropState(
                "driveToThirdSpecimenDrop",
                drive,
                dropper,
                intake,
                robotState
        );
        SpecimenDriveStateConfigurator.configThirdSpecimenDrop(driveToThirdSpecimenDrop);

        DriveToGeneralSpecimenIntakeState driveToFourthSpecimenIntake = new DriveToGeneralSpecimenIntakeState(
                "driveToFourthSpecimenIntake",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configFourthSpecimenIntake(driveToFourthSpecimenIntake);

        DriveToGeneralSpecimenDropState driveToFourthSpecimenDrop = new DriveToGeneralSpecimenDropState(
                "driveToFourthSpecimenDrop",
                drive,
                dropper,
                intake,
                robotState
        );
        SpecimenDriveStateConfigurator.configFourthSpecimenDrop(driveToFourthSpecimenDrop);

        DriveToPark driveToPark = new DriveToPark(
                "driveToPark",
                intake,
                drive,
                dropper,
                distToIntakeTarget(robotState, new Waypoint(114, 9)),
                robotState
        );
        SpecimenDriveStateConfigurator.configDriveToPark(driveToPark);

        EndState endState = new EndState("end");

        // Create the state machine
        stateMachine
                .addState(driveChamberPreload)
                .addState(clipPreload)
                .addState(driveToFirstIntake)
                .addState(grabFirstSample)
                .addState(dropFirstSample)
                .addState(grabSecondSample)
                .addState(dropSecondSample)
                .addState(grabThirdSample)
                .addState(dropThirdSample)
                .addState(driveToFirstSpecimenIntake)
                .addState(intakeSpecimen)
                .addState(clipSpecimen)
                .addState(driveToFirstSpecimenDrop)
                .addState(driveToSecondSpecimenIntake)
                .addState(driveToSecondSpecimenDrop)
                .addState(driveToThirdSpecimenIntake)
                .addState(driveToThirdSpecimenDrop)
                .addState(driveToFourthSpecimenIntake)
                .addState(driveToFourthSpecimenDrop)
                .addState(driveToPark)
                .addState(endState)

                .addTransition(driveChamberPreload, clipPreload, AutoState.DRIVE_END)
                .addTransition(clipPreload, driveToFirstIntake, AutoState.SPECIMEN_PRELOAD_DROP_COMPLETE)
                .addTransition(driveToFirstIntake, grabFirstSample, AutoState.DRIVE_END)
                .addTransition(grabFirstSample, dropFirstSample, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(dropFirstSample, grabSecondSample, AutoState.SAMPLE_DROP_COMPLETE)
                .addTransition(grabSecondSample, dropSecondSample, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(dropSecondSample, grabThirdSample, AutoState.SAMPLE_DROP_COMPLETE)
                .addTransition(grabThirdSample, dropThirdSample, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(dropThirdSample, driveToFirstSpecimenIntake, AutoState.SAMPLE_DROP_COMPLETE)
                .addTransition(driveToFirstSpecimenIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(intakeSpecimen, driveToFirstSpecimenDrop, AutoState.SPECIMEN_1_INTAKE_COMPLETE)
                .addTransition(driveToFirstSpecimenDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(clipSpecimen, driveToSecondSpecimenIntake, AutoState.SPECIMEN_1_DROP_COMPLETE)
                .addTransition(driveToSecondSpecimenIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(intakeSpecimen, driveToSecondSpecimenDrop, AutoState.SPECIMEN_2_INTAKE_COMPLETE)
                .addTransition(driveToSecondSpecimenDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(clipSpecimen, driveToThirdSpecimenIntake, AutoState.SPECIMEN_2_DROP_COMPLETE)
                .addTransition(driveToThirdSpecimenIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(intakeSpecimen, driveToThirdSpecimenDrop, AutoState.SPECIMEN_3_INTAKE_COMPLETE)
                .addTransition(driveToThirdSpecimenDrop, clipSpecimen, AutoState.DRIVE_END)

//                .addTransition(clipSpecimen, driveToFourthSpecimenIntake, AutoState.SPECIMEN_3_DROP_COMPLETE)
//                .addTransition(driveToFourthSpecimenIntake, intakeSpecimen, AutoState.DRIVE_END)
//                .addTransition(intakeSpecimen, driveToFourthSpecimenDrop, AutoState.SPECIMEN_4_INTAKE_COMPLETE)
//                .addTransition(driveToFourthSpecimenDrop, clipSpecimen, AutoState.DRIVE_END)

                .addTransition(clipSpecimen, driveToPark, AutoState.SPECIMEN_3_DROP_COMPLETE)
                .addTransition(driveToPark, endState, AutoState.DRIVE_END)

                .setCurrentState(driveChamberPreload);


        // Register subsystems + Create state machine subsystem
        AutoSubsystem auto = new AutoSubsystem(stateMachine);
        registerSubsystems(auto, drive, odometry, dropper, intake, limelight);
        telemetry.addData("Current X", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Current Y", robotState.getRobotCurrentPose().getY());
        telemetry.addData("Current Heading", Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Expected X", robotState.getRobotFinalPose().getX());
        telemetry.addData("Expected Y", robotState.getRobotFinalPose().getY());
        telemetry.addData("Expected Heading", Math.toDegrees(robotState.getRobotFinalPose().getHeading()));
        telemetry.addData("Expected Slide Position", intake.getTargetPositionInches());
        telemetry.addData("Slide Position", intake.getCurrentSlidePositionInches());
        telemetry.update();
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
        telemetry.addData("Expected Slide Position", intake.getTargetPositionInches());
        telemetry.addData("Slide Position", intake.getCurrentSlidePositionInches());
    }
}
