package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToPreloadDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToSubmersible;
import org.firstinspires.ftc.teamcode.autostates.basket.DropState;
import org.firstinspires.ftc.teamcode.autostates.basket.FirstLevelAscent;
import org.firstinspires.ftc.teamcode.autostates.basket.IntakeSampleState;
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

        // Creating states
        DriveToPreloadDropState driveBasketPreload = new DriveToPreloadDropState(
                "driveToBasketPreload",
                drive,
                dropper,
                robotState);
        BasketDriveStateConfigurator.configPreloadDrop(driveBasketPreload);

        DropState dropSample = new DropState(
                "drop",
                dropper);

        DriveToIntakeState driveIntakeFirstSample = new DriveToIntakeState(
                "driveToIntakeFirstSample",
                drive,
                dropper,
                robotState);
        BasketDriveStateConfigurator.configFirstSampleIntake(driveIntakeFirstSample);

        IntakeSampleState intakeFirstSample = new IntakeSampleState(
                "intakeFirstSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(22.5, 44.75)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));

        DriveToGeneralDropState driveBasketFirstSample = new DriveToGeneralDropState(
                "driveToBasketFirstSample",
                drive,
                dropper,
                intake,
                robotState);
        BasketDriveStateConfigurator.configFirstSampleDrop(driveBasketFirstSample);

        DriveToIntakeState driveIntakeSecondSample = new DriveToIntakeState(
                "driveToIntakeSecondSample",
                drive,
                dropper,
                robotState);
        BasketDriveStateConfigurator.configSecondSampleIntake(driveIntakeSecondSample);

        IntakeSampleState intakeSecondSample = new IntakeSampleState(
                "intakeSecondSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(12.5, 44.75)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));

        DriveToGeneralDropState driveBasketSecondSample = new DriveToGeneralDropState(
                "driveToBasketSecondSample",
                drive,
                dropper,
                intake,
                robotState);
        BasketDriveStateConfigurator.configSecondSampleDrop(driveBasketSecondSample);

        DriveToIntakeState driveIntakeThirdSample = new DriveToIntakeState(
                "driveToIntakeThirdSample",
                drive,
                dropper,
                robotState);
        BasketDriveStateConfigurator.configThirdSampleIntake(driveIntakeThirdSample);

        IntakeSampleState intakeThirdSample = new IntakeSampleState(
                "intakeThirdSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(2.5, 44.75)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));

        DriveToGeneralDropState driveBasketThirdSample = new DriveToGeneralDropState(
                "driveToBasketThirdSample",
                drive,
                dropper,
                intake,
                robotState);
        BasketDriveStateConfigurator.configThirdSampleDrop(driveBasketThirdSample);

        DriveToSubmersible driveToSubmersible = new DriveToSubmersible(
                "driveToSubmersible",
                drive,
                dropper,
                intake,
                robotState
        );
        BasketDriveStateConfigurator.configDriveToSubmersible(driveToSubmersible);

        FirstLevelAscent firstLevelAscent = new FirstLevelAscent(
                "firstLevelAscent",
                dropper
        );


        // Create the state machine
        stateMachine
                .addState(driveBasketPreload)
                .addState(dropSample)
                .addState(driveIntakeFirstSample)
                .addState(intakeFirstSample)
                .addState(driveBasketFirstSample)
                .addState(driveIntakeSecondSample)
                .addState(intakeSecondSample)
                .addState(driveBasketSecondSample)
                .addState(driveIntakeThirdSample)
                .addState(intakeThirdSample)
                .addState(driveBasketThirdSample)
                .addState(driveToSubmersible)
                .addState(firstLevelAscent)

                .addTransition(driveBasketPreload, dropSample, AutoState.DRIVE_END)
                .addTransition(driveBasketPreload, dropSample, AutoState.TIMEOUT)
                .addTransition(dropSample, driveIntakeFirstSample, AutoState.SAMPLE_0_DROP_COMPLETE)
                .addTransition(driveIntakeFirstSample, intakeFirstSample, AutoState.DRIVE_END)
                .addTransition(driveIntakeFirstSample, intakeFirstSample, AutoState.TIMEOUT)
                .addTransition(intakeFirstSample, driveBasketFirstSample, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(driveBasketFirstSample, dropSample, AutoState.DRIVE_END)
                .addTransition(driveBasketFirstSample, dropSample, AutoState.TIMEOUT)
                .addTransition(dropSample, driveIntakeSecondSample, AutoState.SAMPLE_1_DROP_COMPLETE)
                .addTransition(driveIntakeSecondSample, intakeSecondSample, AutoState.DRIVE_END)
                .addTransition(driveIntakeSecondSample, intakeSecondSample, AutoState.TIMEOUT)
                .addTransition(intakeSecondSample, driveBasketSecondSample, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(driveBasketSecondSample, dropSample, AutoState.DRIVE_END)
                .addTransition(driveBasketSecondSample, dropSample, AutoState.TIMEOUT)
                .addTransition(dropSample, driveIntakeThirdSample, AutoState.SAMPLE_2_DROP_COMPLETE)
                .addTransition(driveIntakeThirdSample, intakeThirdSample, AutoState.DRIVE_END)
                .addTransition(driveIntakeThirdSample, intakeThirdSample, AutoState.TIMEOUT)
                .addTransition(intakeThirdSample, driveBasketThirdSample, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(driveBasketThirdSample, dropSample, AutoState.DRIVE_END)
                .addTransition(driveBasketThirdSample, dropSample, AutoState.TIMEOUT)
                .addTransition(dropSample, driveToSubmersible, AutoState.SAMPLE_3_DROP_COMPLETE)
                .addTransition(driveToSubmersible, firstLevelAscent, AutoState.DRIVE_END)
                .addTransition(driveToSubmersible, firstLevelAscent, AutoState.TIMEOUT)

                .setCurrentState(driveBasketPreload);

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
        registerSubsystems(auto, drive, odometry, dropper);
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
