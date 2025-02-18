package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autostates.specimen.ClipPreloadState;
import org.firstinspires.ftc.teamcode.autostates.specimen.ClipSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenDropState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPark;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPoseState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.EndState;
import org.firstinspires.ftc.teamcode.autostates.specimen.PickupSpecimenState;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import java.util.function.DoubleSupplier;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.statemachine.StateMachine;
import team.techtigers.core.paths.Waypoint;

@Config
@Autonomous
public class SpecimenAutoOpMode extends BaseOpMode {
    private RobotState robotState;
    private IntakeSubsystem intake;
    private DropperSubsystem dropper;

    private DoubleSupplier distToIntakeTarget(RobotState robotState, Waypoint target) {
        return () -> Math.min(Math.hypot(target.getX() - robotState.getRobotCurrentPose().getX(),
                target.getY() - robotState.getRobotCurrentPose().getY()) - 9, 19);
    }

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        StateMachine<AutoState> stateMachine = new StateMachine<>();
        robotState = new RobotState(true, true);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize subsystems
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap,
                robotState, new Waypoint(79.25, 7.25, Math.toRadians(90)));
        dropper = new DropperSubsystem(hardwareMap,
                robotState);
        intake = new IntakeSubsystem(hardwareMap, robotState);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);

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
                drive,
                robotState);

        DriveToFirstIntakeState driveToFirstIntake = new DriveToFirstIntakeState(
                "driveToFirstIntake",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configFirstIntake(driveToFirstIntake);

        DriveToPoseState firstPush = new DriveToPoseState(
                "firstPush",
                drive,
                robotState,
                3
        );
        SpecimenDriveStateConfigurator.configFirstPush(firstPush);

        DriveToPoseState secondIntake = new DriveToPoseState(
                "secondIntake",
                drive,
                robotState
                , 3
        );
        SpecimenDriveStateConfigurator.configSecondIntake(secondIntake);

        DriveToPoseState secondPush = new DriveToPoseState(
                "secondPush",
                drive,
                robotState,
                3
        );
        SpecimenDriveStateConfigurator.configSecondPush(secondPush);

        DriveToPoseState thirdIntake = new DriveToPoseState(
                "thirdIntake",
                drive,
                robotState,
                3
        );
        SpecimenDriveStateConfigurator.configThirdIntake(thirdIntake);

        DriveToPoseState thirdPush = new DriveToPoseState(
                "thirdPush",
                drive,
                robotState,
                3
        );
        SpecimenDriveStateConfigurator.configThirdPush(thirdPush);

        DriveToPoseState driveToFirstSpecimenIntake = new DriveToPoseState(
                "driveToFirstSpecimenIntake",
                drive,
                robotState,
                2
        );
        SpecimenDriveStateConfigurator.configFirstSpecimenIntake(driveToFirstSpecimenIntake);

        ClipSpecimenState clipSpecimen = new ClipSpecimenState(
                "clipSpecimen",
                dropper,
                drive,
                robotState
        );

        PickupSpecimenState intakeSpecimen = new PickupSpecimenState(
                "intakeSpecimen",
                drive,
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
        SpecimenDriveStateConfigurator.configGeneralSpecimenIntake(driveToSecondSpecimenIntake);

        DriveToGeneralSpecimenDropState driveToSecondSpecimenDrop = new DriveToGeneralSpecimenDropState(
                "driveToSecondSpecimenDrop",
                drive,
                dropper,
                intake,
                robotState
        );
        SpecimenDriveStateConfigurator.configGeneralSpecimenDrop(driveToSecondSpecimenDrop);

        DriveToGeneralSpecimenIntakeState driveToThirdSpecimenIntake = new DriveToGeneralSpecimenIntakeState(
                "driveToThirdSpecimenIntake",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configGeneralSpecimenIntake(driveToThirdSpecimenIntake);

        DriveToGeneralSpecimenDropState driveToThirdSpecimenDrop = new DriveToGeneralSpecimenDropState(
                "driveToThirdSpecimenDrop",
                drive,
                dropper,
                intake,
                robotState
        );
        SpecimenDriveStateConfigurator.configGeneralSpecimenDrop(driveToThirdSpecimenDrop);

        DriveToGeneralSpecimenIntakeState driveToFourthSpecimenIntake = new DriveToGeneralSpecimenIntakeState(
                "driveToFourthSpecimenIntake",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configGeneralSpecimenIntake(driveToFourthSpecimenIntake);

        DriveToGeneralSpecimenDropState driveToFourthSpecimenDrop = new DriveToGeneralSpecimenDropState(
                "driveToFourthSpecimenDrop",
                drive,
                dropper,
                intake,
                robotState
        );
        SpecimenDriveStateConfigurator.configGeneralSpecimenDrop(driveToFourthSpecimenDrop);

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
                .addState(firstPush)
                .addState(secondIntake)
                .addState(secondPush)
                .addState(thirdIntake)
                .addState(thirdPush)
                .addState(driveToFirstSpecimenIntake)
                .addState(clipSpecimen)
                .addState(intakeSpecimen)
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
                .addTransition(driveChamberPreload, clipPreload, AutoState.TIMEOUT)

//                .addTransition(clipPreload, endState, AutoState.SPECIMEN_PRELOAD_DROP_COMPLETE)

                .addTransition(clipPreload, driveToFirstIntake, AutoState.SPECIMEN_PRELOAD_DROP_COMPLETE)

//                .addTransition(driveToFirstIntake, endState, AutoState.DRIVE_END)

                .addTransition(driveToFirstIntake, firstPush, AutoState.DRIVE_END)
                .addTransition(driveToFirstIntake, firstPush, AutoState.TIMEOUT)
                .addTransition(firstPush, secondIntake, AutoState.DRIVE_END)
                .addTransition(firstPush, secondIntake, AutoState.TIMEOUT)
                .addTransition(secondIntake, secondPush, AutoState.DRIVE_END)
                .addTransition(secondIntake, secondPush, AutoState.TIMEOUT)
                .addTransition(secondPush, thirdIntake, AutoState.DRIVE_END)
                .addTransition(secondPush, thirdIntake, AutoState.TIMEOUT)
                .addTransition(thirdIntake, thirdPush, AutoState.DRIVE_END)
                .addTransition(thirdIntake, thirdPush, AutoState.TIMEOUT)
//                .addTransition(thirdPush, endState, AutoState.DRIVE_END)
//                .addTransition(thirdPush, endState, AutoState.TIMEOUT)
                .addTransition(thirdPush, driveToFirstSpecimenIntake, AutoState.DRIVE_END)
                .addTransition(thirdPush, driveToFirstSpecimenIntake, AutoState.TIMEOUT)

                .addTransition(driveToFirstSpecimenIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFirstSpecimenIntake, intakeSpecimen, AutoState.TIMEOUT)

//                .addTransition(driveToFirstSpecimenIntake, endState, AutoState.DRIVE_END)

                .addTransition(intakeSpecimen, driveToFirstSpecimenDrop, AutoState.SPECIMEN_1_INTAKE_COMPLETE)

                .addTransition(driveToFirstSpecimenDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFirstSpecimenDrop, clipSpecimen, AutoState.TIMEOUT)
                .addTransition(clipSpecimen, driveToSecondSpecimenIntake, AutoState.SPECIMEN_1_DROP_COMPLETE)
                .addTransition(driveToSecondSpecimenIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToSecondSpecimenIntake, intakeSpecimen, AutoState.TIMEOUT)

                .addTransition(intakeSpecimen, driveToSecondSpecimenDrop, AutoState.SPECIMEN_2_INTAKE_COMPLETE)
                .addTransition(driveToSecondSpecimenDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToSecondSpecimenDrop, clipSpecimen, AutoState.TIMEOUT)
                .addTransition(clipSpecimen, driveToThirdSpecimenIntake, AutoState.SPECIMEN_2_DROP_COMPLETE)
                .addTransition(driveToThirdSpecimenIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToThirdSpecimenIntake, intakeSpecimen, AutoState.TIMEOUT)
                .addTransition(intakeSpecimen, driveToThirdSpecimenDrop, AutoState.SPECIMEN_3_INTAKE_COMPLETE)
                .addTransition(driveToThirdSpecimenDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToThirdSpecimenDrop, clipSpecimen, AutoState.TIMEOUT)

                .addTransition(clipSpecimen, driveToFourthSpecimenIntake, AutoState.SPECIMEN_3_DROP_COMPLETE)
                .addTransition(driveToFourthSpecimenIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFourthSpecimenIntake, intakeSpecimen, AutoState.TIMEOUT)
                .addTransition(intakeSpecimen, driveToFourthSpecimenDrop, AutoState.SPECIMEN_4_INTAKE_COMPLETE)
                .addTransition(driveToFourthSpecimenDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFourthSpecimenDrop, clipSpecimen, AutoState.TIMEOUT)

//                .addTransition(clipSpecimen, endState, AutoState.SPECIMEN_4_DROP_COMPLETE)

                .addTransition(clipSpecimen, driveToPark, AutoState.SPECIMEN_4_DROP_COMPLETE)

                .addTransition(driveToPark, endState, AutoState.DRIVE_END)
                .addTransition(driveToPark, endState, AutoState.TIMEOUT)

//                .setCurrentState(driveChamberPreload);
                .setCurrentState(driveChamberPreload);


        // Register subsystems + Create state machine subsystem
        AutoSubsystem auto = new AutoSubsystem(stateMachine);
        registerSubsystems(auto, drive, odometry, dropper, intake, limelight, sensor);
        telemetry.addData("Current X", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Current Y", robotState.getRobotCurrentPose().getY());
        telemetry.addData("Current Heading", Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Expected X", robotState.getRobotFinalPose().getX());
        telemetry.addData("Expected Y", robotState.getRobotFinalPose().getY());
        telemetry.addData("Expected Heading", Math.toDegrees(robotState.getRobotFinalPose().getHeading()));
//        telemetry.addData("Expected Slide Position", intake.getTargetPositionInches());
//        telemetry.addData("Slide Position", intake.getCurrentSlidePositionInches());
        telemetry.addLine();
        telemetry.addData("Expected Dropper Slide Position", dropper.getTargetPositionInches());
        telemetry.addData("Dropper Slide Position", dropper.getCurrentSlidePositionInches());
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
        telemetry.addLine();
//        telemetry.addData("Expected Intake Slide Position", intake.getTargetPositionInches());
//        telemetry.addData("Intake Slide Position", intake.getCurrentSlidePositionInches());
        telemetry.addLine();
        telemetry.addData("Expected Dropper Slide Position", dropper.getTargetPositionInches());
        telemetry.addData("Dropper Slide Position", dropper.getCurrentSlidePositionInches());
    }
}
