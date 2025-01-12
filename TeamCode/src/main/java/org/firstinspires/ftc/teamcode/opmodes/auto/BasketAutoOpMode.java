package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autostates.DriveToGeneralDropState;
import org.firstinspires.ftc.teamcode.autostates.DriveToSubmersible;
import org.firstinspires.ftc.teamcode.autostates.DropState;
import org.firstinspires.ftc.teamcode.autostates.DriveToPreloadDropState;
import org.firstinspires.ftc.teamcode.autostates.FirstLevelAscent;
import org.firstinspires.ftc.teamcode.autostates.IntakeSampleState;
import org.firstinspires.ftc.teamcode.autostates.DriveToIntakeState;
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

@Config
@Autonomous
public class BasketAutoOpMode extends BaseOpMode {
    private RobotState robotState;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private DoubleSupplier distToIntakeTarget(RobotState robotState, Waypoint target) {
        return () -> Math.min(Math.hypot(target.getX() - robotState.getRobotCurrentPose().getX(),
                target.getY() - robotState.getRobotCurrentPose().getY()) - 9, 19);
    }

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        StateMachine<AutoState> stateMachine = new StateMachine<>();
        robotState = new RobotState(true, true);

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
        DriveStateConfigurator.configPreloadDrop(driveBasketPreload);

        DropState dropSample = new DropState(
                "drop",
                dropper);

        DriveToIntakeState driveIntakeFirstSample = new DriveToIntakeState(
                "driveToIntakeFirstSample",
                drive,
                dropper,
                robotState);
        DriveStateConfigurator.configFirstSampleIntake(driveIntakeFirstSample);

        IntakeSampleState intakeFirstSample = new IntakeSampleState(
                "intakeFirstSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(23, 45)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));

        DriveToGeneralDropState driveBasketFirstSample = new DriveToGeneralDropState(
                "driveToBasketFirstSample",
                drive,
                dropper,
                intake,
                robotState);
        DriveStateConfigurator.configFirstSampleDrop(driveBasketFirstSample);

        DriveToIntakeState driveIntakeSecondSample = new DriveToIntakeState(
                "driveToIntakeSecondSample",
                drive,
                dropper,
                robotState);
        DriveStateConfigurator.configSecondSampleIntake(driveIntakeSecondSample);

        IntakeSampleState intakeSecondSample = new IntakeSampleState(
                "intakeSecondSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(13, 45)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));

        DriveToGeneralDropState driveBasketSecondSample = new DriveToGeneralDropState(
                "driveToBasketSecondSample",
                drive,
                dropper,
                intake,
                robotState);
        DriveStateConfigurator.configSecondSampleDrop(driveBasketSecondSample);

        DriveToIntakeState driveIntakeThirdSample = new DriveToIntakeState(
                "driveToIntakeThirdSample",
                drive,
                dropper,
                robotState);
        DriveStateConfigurator.configThirdSampleIntake(driveIntakeThirdSample);

        IntakeSampleState intakeThirdSample = new IntakeSampleState(
                "intakeThirdSample",
                intake,
                dropper,
                robotState,
                distToIntakeTarget(robotState, new Waypoint(3, 45)),
                () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));

        DriveToGeneralDropState driveBasketThirdSample = new DriveToGeneralDropState(
                "driveToBasketThirdSample",
                drive,
                dropper,
                intake,
                robotState);
        DriveStateConfigurator.configThirdSampleDrop(driveBasketThirdSample);

        DriveToSubmersible driveToSubmersible = new DriveToSubmersible(
                "driveToSubmersible",
                drive,
                dropper,
                intake,
                robotState
        );
        DriveStateConfigurator.configDriveToSubmersible(driveToSubmersible);

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
                .addTransition(dropSample, driveIntakeFirstSample, AutoState.SAMPLE_0_DROP_COMPLETE)
//                .addTransition(driveIntakeFirstSample, intakeFirstSample, AutoState.DRIVE_END)
                .addTransition(intakeFirstSample, driveBasketFirstSample, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(driveBasketFirstSample, dropSample, AutoState.DRIVE_END)
                .addTransition(dropSample, driveIntakeSecondSample, AutoState.SAMPLE_1_DROP_COMPLETE)
                .addTransition(driveIntakeSecondSample, intakeSecondSample, AutoState.DRIVE_END)
                .addTransition(intakeSecondSample, driveBasketSecondSample, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(driveBasketSecondSample, dropSample, AutoState.DRIVE_END)
                .addTransition(dropSample, driveIntakeThirdSample, AutoState.SAMPLE_2_DROP_COMPLETE)
                .addTransition(driveIntakeThirdSample, intakeThirdSample, AutoState.DRIVE_END)
                .addTransition(intakeThirdSample, driveBasketThirdSample, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(driveBasketThirdSample, dropSample, AutoState.DRIVE_END)
                .addTransition(dropSample, driveToSubmersible, AutoState.SAMPLE_3_DROP_COMPLETE)
                .addTransition(driveToSubmersible, firstLevelAscent, AutoState.DRIVE_END)

                .setCurrentState(driveBasketPreload);


        // Register subsystems + Create state machine subsystem
        AutoSubsystem auto = new AutoSubsystem(stateMachine);
        registerSubsystems(auto, drive, odometry, dropper);

        update();
        telemetry.update();
    }

    @Override
    public void update() {
        telemetry.addData("Current Pose", robotState.getRobotCurrentPose());
        telemetry.addData("Final Pose", robotState.getRobotFinalPose());
        telemetry.addLine();
        telemetry.addData("Current X", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Current Y", robotState.getRobotCurrentPose().getY());
        telemetry.addData("Current Heading", robotState.getRobotCurrentPose().getHeading());
        telemetry.addLine();
        telemetry.addData("Expected X", robotState.getRobotFinalPose().getX());
        telemetry.addData("Expected Y", robotState.getRobotFinalPose().getY());
        telemetry.addData("Expected Heading", robotState.getRobotFinalPose().getHeading());
    }
}
