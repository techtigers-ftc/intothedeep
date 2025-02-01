package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.VisionSamplePickupState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSampleDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSampleIntakeState;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToPreloadDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToSubmersible;
import org.firstinspires.ftc.teamcode.autostates.basket.DropState;
import org.firstinspires.ftc.teamcode.autostates.basket.IntakeSampleState;
import org.firstinspires.ftc.teamcode.autostates.specimen.EndState;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import java.util.function.DoubleSupplier;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.statemachine.StateMachine;
import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.utils.RobotSaveState;

@Config
public abstract class BasketAutoOpMode extends BaseOpMode {
    private RobotState robotState;

    protected abstract boolean isBlue();

    private DoubleSupplier distToIntakeTarget(RobotState robotState, Waypoint target) {
        return () -> Math.min(Math.hypot(target.getX() - robotState.getRobotCurrentPose().getX(),
                target.getY() - robotState.getRobotCurrentPose().getY()) - 8.75, IntakeSubsystem.SLIDES_MAX);
    }

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        StateMachine<AutoState> stateMachine = new StateMachine<>();
        robotState = new RobotState(isBlue(), true);

        RobotSaveState.reset();

        // Initialize subsystems
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap,
                robotState, new Waypoint(29.75, 7.25, Math.toRadians(90)));
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap,
                robotState);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);

        // Creating states
        DriveToPreloadDropState driveToPreloadDrop = new DriveToPreloadDropState(
                "driveToPreloadDrop",
                drive,
                dropper,
                robotState);
        BasketDriveStateConfigurator.configPreloadDrop(driveToPreloadDrop);

        DropState dropSample = new DropState(
                "drop",
                dropper);

        DriveToGeneralSampleIntakeState driveToFirstIntake = new DriveToGeneralSampleIntakeState(
                "driveToFirstIntake",
                drive,
                dropper,
                robotState);
        BasketDriveStateConfigurator.configFirstSampleIntake(driveToFirstIntake);

        IntakeSampleState intakeFirstSample = new IntakeSampleState(
                "intakeFirstSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(22.5, 44.75)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));

        DriveToGeneralSampleDropState driveToFirstDrop = new DriveToGeneralSampleDropState(
                "driveToFirstDrop",
                drive,
                dropper,
                intake,
                robotState);
        BasketDriveStateConfigurator.configFirstSampleDrop(driveToFirstDrop);

        DriveToGeneralSampleIntakeState driveToSecondIntake = new DriveToGeneralSampleIntakeState(
                "driveToSecondIntake",
                drive,
                dropper,
                robotState);
        BasketDriveStateConfigurator.configSecondSampleIntake(driveToSecondIntake);

        IntakeSampleState intakeSecondSample = new IntakeSampleState(
                "intakeSecondSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(12.5, 44.75)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));

        DriveToGeneralSampleDropState driveToSecondDrop = new DriveToGeneralSampleDropState(
                "driveToSecondDrop",
                drive,
                dropper,
                intake,
                robotState);
        BasketDriveStateConfigurator.configSecondSampleDrop(driveToSecondDrop);

        DriveToGeneralSampleIntakeState driveToThirdIntake = new DriveToGeneralSampleIntakeState(
                "driveToThirdIntake",
                drive,
                dropper,
                robotState);
        BasketDriveStateConfigurator.configThirdSampleIntake(driveToThirdIntake);

        IntakeSampleState intakeThirdSample = new IntakeSampleState(
                "intakeThirdSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(2.5, 44.75)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));

        DriveToGeneralSampleDropState driveToThirdDrop = new DriveToGeneralSampleDropState(
                "driveToThirdDrop",
                drive,
                dropper,
                intake,
                robotState);
        BasketDriveStateConfigurator.configThirdSampleDrop(driveToThirdDrop);

        VisionSamplePickupState visionPickupSample = new VisionSamplePickupState(
                "visionPickupSample",
                intake,
                dropper,
                drive,
                robotState
        );

        DriveToGeneralSampleIntakeState driveToFourthIntake = new DriveToGeneralSampleIntakeState(
                "driveToFourthIntake",
                drive,
                dropper,
                robotState
        );
        BasketDriveStateConfigurator.configFourthSampleIntake(driveToFourthIntake);

        DriveToGeneralSampleDropState driveToFourthDrop = new DriveToGeneralSampleDropState(
                "driveToFourthDrop",
                drive,
                dropper,
                intake,
                robotState
        );
        BasketDriveStateConfigurator.configFourthSampleDrop(driveToFourthDrop);

        DriveToGeneralSampleIntakeState driveToFifthIntake = new DriveToGeneralSampleIntakeState(
                "driveToFifthIntake",
                drive,
                dropper,
                robotState
        );
        BasketDriveStateConfigurator.configFifthSampleIntake(driveToFifthIntake);

        DriveToGeneralSampleDropState driveToFifthDrop = new DriveToGeneralSampleDropState(
                "driveToFifthDrop",
                drive,
                dropper,
                intake,
                robotState
        );
        BasketDriveStateConfigurator.configFifthSampleDrop(driveToFifthDrop);

        DriveToSubmersible driveToSubmersible = new DriveToSubmersible(
                "driveToSubmersible",
                drive,
                dropper,
                intake,
                robotState
        );
        BasketDriveStateConfigurator.configDriveToSubmersible(driveToSubmersible);

        EndState endState = new EndState("endState");

        // Create the state machine
        stateMachine
                .addState(driveToPreloadDrop)
                .addState(dropSample)
                .addState(driveToFirstIntake)
                .addState(intakeFirstSample)
                .addState(driveToFirstDrop)
                .addState(driveToSecondIntake)
                .addState(intakeSecondSample)
                .addState(driveToSecondDrop)
                .addState(driveToThirdIntake)
                .addState(intakeThirdSample)
                .addState(driveToThirdDrop)
                .addState(visionPickupSample)
                .addState(driveToFourthIntake)
                .addState(driveToFourthDrop)
                .addState(driveToFifthIntake)
                .addState(driveToFifthDrop)
                .addState(driveToSubmersible)
                .addState(endState)

                .addTransition(driveToPreloadDrop, dropSample, AutoState.DRIVE_END)
                .addTransition(dropSample, driveToFirstIntake, AutoState.SAMPLE_PRELOAD_DROP_COMPLETE)
                .addTransition(driveToFirstIntake, intakeFirstSample, AutoState.DRIVE_END)
                .addTransition(intakeFirstSample, driveToFirstDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(driveToFirstDrop, dropSample, AutoState.DRIVE_END)
                .addTransition(dropSample, driveToSecondIntake, AutoState.SAMPLE_1_DROP_COMPLETE)
                .addTransition(driveToSecondIntake, intakeSecondSample, AutoState.DRIVE_END)
                .addTransition(intakeSecondSample, driveToSecondDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(driveToSecondDrop, dropSample, AutoState.DRIVE_END)
                .addTransition(dropSample, driveToThirdIntake, AutoState.SAMPLE_2_DROP_COMPLETE)
                .addTransition(driveToThirdIntake, intakeThirdSample, AutoState.DRIVE_END)
                .addTransition(intakeThirdSample, driveToThirdDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(driveToThirdDrop, dropSample, AutoState.DRIVE_END)
                .addTransition(dropSample, driveToFourthIntake, AutoState.SAMPLE_3_DROP_COMPLETE)
                .addTransition(driveToFourthIntake, visionPickupSample, AutoState.DRIVE_END)
                .addTransition(visionPickupSample, driveToFourthDrop, AutoState.SAMPLE_4_INTAKE_COMPLETE)
                .addTransition(driveToFourthDrop, dropSample, AutoState.DRIVE_END)
                .addTransition(dropSample, driveToFifthIntake, AutoState.SAMPLE_4_DROP_COMPLETE)
                .addTransition(driveToFifthIntake, visionPickupSample, AutoState.DRIVE_END)
                .addTransition(visionPickupSample, driveToFifthDrop, AutoState.SAMPLE_5_INTAKE_COMPLETE)
                .addTransition(driveToFifthDrop, dropSample, AutoState.DRIVE_END)
                .addTransition(dropSample, driveToSubmersible, AutoState.SAMPLE_5_DROP_COMPLETE)
                .addTransition(driveToSubmersible, firstLevelAscent, AutoState.DRIVE_END)

                .setCurrentState(driveToPreloadDrop);

        telemetry.addData("Current X", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Current Y", robotState.getRobotCurrentPose().getY());
        telemetry.addData("Current Heading", Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Expected X", robotState.getRobotFinalPose().getX());
        telemetry.addData("Expected Y", robotState.getRobotFinalPose().getY());
        telemetry.addData("Expected Heading", Math.toDegrees(robotState.getRobotFinalPose().getHeading()));
        telemetry.update();

        // Register subsystems + Create state machine subsystem
        AutoSubsystem auto = new AutoSubsystem(stateMachine);
        registerSubsystems(auto, drive, odometry, dropper, sensor);
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
    }

    @Override
    public void end() {
        RobotSaveState.getInstance().setState("robotCurrentPose", robotState.getRobotCurrentPose());
    }
}
