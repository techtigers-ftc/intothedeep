package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.autostates.EndState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveFromSubmersibleSampleDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSampleDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSampleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSubmersibleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToPreloadDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToSubmersible;
import org.firstinspires.ftc.teamcode.autostates.basket.FailedIntakeSampleState;
import org.firstinspires.ftc.teamcode.autostates.basket.FailedSubmersibleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.SampleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.SubmersibleIntakeState;
import org.firstinspires.ftc.teamcode.display.view.AutoView;
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
import team.techtigers.base.visualdisplay.AdafruitNeoPixel;
import team.techtigers.base.visualdisplay.VisualDisplaySubsystem;
import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.utils.RobotSaveState;

@Config
public abstract class BasketAutoOpMode extends BaseOpMode {
    private RobotState robotState;
    private IntakeSubsystem intake;
    private DropperSubsystem dropper;

    protected abstract boolean isBlue();

    private DoubleSupplier distToIntakeTarget(RobotState robotState, Waypoint target) {
        return () -> Math.min(Math.hypot(target.getX() - robotState.getRobotCurrentPose().getX(),
                target.getY() - robotState.getRobotCurrentPose().getY()) - 9.5, IntakeSubsystem.SLIDES_MAX);
    }

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        StateMachine<AutoState> stateMachine = new StateMachine<>();
        robotState = new RobotState(isBlue(), true);

        RobotSaveState.reset();

        // Initialize subsystems
        dropper = new DropperSubsystem(hardwareMap,
                robotState);
        intake = new IntakeSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap,
                robotState, new Waypoint(29.75, 7.25, Math.toRadians(90)));
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);

        AdafruitNeoPixel displayDriver = hardwareMap.get(AdafruitNeoPixel.class, "visual_display");
        displayDriver.initialize(224, 3);
        VisualDisplaySubsystem visualDisplaySubsystem = new VisualDisplaySubsystem(displayDriver, new AutoView(robotState));

        // Creating states
        DriveToPreloadDropState driveToPreloadDrop = new DriveToPreloadDropState(
                "driveToPreloadDrop",
                drive,
                dropper,
                intake,
                15,
                robotState);
        BasketDriveStateConfigurator.configPreloadDrop(driveToPreloadDrop);

        DriveToGeneralSampleIntakeState driveToFirstIntake = new DriveToGeneralSampleIntakeState(
                "driveToFirstIntake",
                drive,
                dropper,
                intake,
                robotState);
        BasketDriveStateConfigurator.configFirstSampleIntake(driveToFirstIntake);

        SampleIntakeState intakeFirstSample = new SampleIntakeState(
                "intakeFirstSample",
                drive,
                intake,
                dropper,
                robotState
        );

        FailedIntakeSampleState failedIntakeSample = new FailedIntakeSampleState(
                "failedIntakeSample",
                drive,
                intake,
                dropper,
                robotState
        );

        DriveToGeneralSampleDropState driveToFirstDrop = new DriveToGeneralSampleDropState(
                "driveToFirstDrop",
                drive,
                dropper,
                intake,
                15,
                robotState);
        BasketDriveStateConfigurator.configFirstSampleDrop(driveToFirstDrop);

        DriveToGeneralSampleIntakeState driveToSecondIntake = new DriveToGeneralSampleIntakeState(
                "driveToSecondIntake",
                drive,
                dropper,
                intake,
                robotState);
        BasketDriveStateConfigurator.configSecondSampleIntake(driveToSecondIntake);

        SampleIntakeState intakeSecondSample = new SampleIntakeState(
                "intakeSecondSample",
                drive,
                intake,
                dropper,
                robotState
        );

        DriveToGeneralSampleDropState driveToSecondDrop = new DriveToGeneralSampleDropState(
                "driveToSecondDrop",
                drive,
                dropper,
                intake,
                16,
                robotState);
        BasketDriveStateConfigurator.configSecondSampleDrop(driveToSecondDrop);

        DriveToGeneralSampleIntakeState driveToThirdIntake = new DriveToGeneralSampleIntakeState(
                "driveToThirdIntake",
                drive,
                dropper,
                intake,
                robotState);
        BasketDriveStateConfigurator.configThirdSampleIntake(driveToThirdIntake);

        SampleIntakeState intakeThirdSample = new SampleIntakeState(
                "intakeThirdSample",
                drive,
                intake,
                dropper,
                robotState
        );

        DriveToGeneralSampleDropState driveToThirdDrop = new DriveToGeneralSampleDropState(
                "driveToThirdDrop",
                drive,
                dropper,
                intake,
                0,
                robotState);
        BasketDriveStateConfigurator.configThirdSampleDrop(driveToThirdDrop);

        SubmersibleIntakeState intakeFourthSample = new SubmersibleIntakeState(
                "intakeFourthSample",
                drive,
                intake,
                dropper,
                robotState
        );

        FailedSubmersibleIntakeState failedIntakeSubmersible = new FailedSubmersibleIntakeState(
                "failedIntakeSubmersible",
                drive,
                intake,
                dropper,
                robotState
        );

        DriveToGeneralSubmersibleIntakeState driveToFourthIntake = new DriveToGeneralSubmersibleIntakeState(
                "driveToFourthIntake",
                drive,
                intake,
                dropper,
                robotState
        );
        BasketDriveStateConfigurator.configFourthSampleIntake(driveToFourthIntake);

        DriveFromSubmersibleSampleDropState driveToFourthDrop = new DriveFromSubmersibleSampleDropState(
                "driveToFourthDrop",
                drive,
                dropper,
                intake,
                robotState
        );
        BasketDriveStateConfigurator.configFourthSampleDrop(driveToFourthDrop);

        SubmersibleIntakeState intakeFifthSample = new SubmersibleIntakeState(
                "intakeFifthSample",
                drive,
                intake,
                dropper,
                robotState
        );

        DriveToGeneralSubmersibleIntakeState driveToFifthIntake = new DriveToGeneralSubmersibleIntakeState(
                "driveToFifthIntake",
                drive,
                intake,
                dropper,
                robotState
        );
        BasketDriveStateConfigurator.configFifthSampleIntake(driveToFifthIntake);

        DriveFromSubmersibleSampleDropState driveToFifthDrop = new DriveFromSubmersibleSampleDropState(
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
                .addState(driveToFirstIntake)
                .addState(intakeFirstSample)
                .addState(failedIntakeSample)
                .addState(driveToFirstDrop)
                .addState(driveToSecondIntake)
                .addState(intakeSecondSample)
                .addState(driveToSecondDrop)
                .addState(driveToThirdIntake)
                .addState(intakeThirdSample)
                .addState(driveToThirdDrop)
                .addState(intakeFourthSample)
                .addState(failedIntakeSubmersible)
                .addState(driveToFourthIntake)
                .addState(driveToFourthDrop)
                .addState(intakeFifthSample)
                .addState(driveToFifthIntake)
                .addState(driveToFifthDrop)
                .addState(driveToSubmersible)
                .addState(endState)

                .addTransition(driveToPreloadDrop, driveToFirstIntake, AutoState.DRIVE_END)
                .addTransition(driveToPreloadDrop, driveToFirstIntake, AutoState.TIMEOUT)

                .addTransition(driveToFirstIntake, intakeFirstSample, AutoState.DRIVE_END)
                .addTransition(driveToFirstIntake, intakeFirstSample, AutoState.TIMEOUT)

                .addTransition(intakeFirstSample, failedIntakeSample, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(intakeFirstSample, driveToFirstDrop, AutoState.TIMEOUT)
                .addTransition(intakeFirstSample, driveToFirstDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(failedIntakeSample, failedIntakeSample, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(failedIntakeSample, driveToFirstDrop, AutoState.SAMPLE_1_INTAKE_RECOVERED)
                .addTransition(failedIntakeSample, driveToFirstDrop, AutoState.TIMEOUT)

                .addTransition(driveToFirstDrop, driveToSecondIntake, AutoState.DRIVE_END)
                .addTransition(driveToFirstDrop, driveToSecondIntake, AutoState.TIMEOUT)

                .addTransition(driveToSecondIntake, intakeSecondSample, AutoState.DRIVE_END)
                .addTransition(driveToSecondIntake, intakeSecondSample, AutoState.TIMEOUT)

                .addTransition(intakeSecondSample, failedIntakeSample, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(intakeSecondSample, driveToSecondDrop, AutoState.TIMEOUT)
                .addTransition(intakeSecondSample, driveToSecondDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(failedIntakeSample, driveToSecondDrop, AutoState.SAMPLE_2_INTAKE_RECOVERED)
                .addTransition(failedIntakeSample, driveToSecondDrop, AutoState.TIMEOUT)

                .addTransition(driveToSecondDrop, driveToThirdIntake, AutoState.DRIVE_END)
                .addTransition(driveToSecondDrop, driveToThirdIntake, AutoState.TIMEOUT)

                .addTransition(driveToThirdIntake, intakeThirdSample, AutoState.DRIVE_END)
                .addTransition(driveToThirdIntake, intakeThirdSample, AutoState.TIMEOUT)

                .addTransition(intakeThirdSample, failedIntakeSample, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(intakeThirdSample, driveToThirdDrop, AutoState.TIMEOUT)
                .addTransition(intakeThirdSample, driveToThirdDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(failedIntakeSample, driveToThirdDrop, AutoState.SAMPLE_3_INTAKE_RECOVERED)
                .addTransition(failedIntakeSample, driveToThirdDrop, AutoState.TIMEOUT)

                .addTransition(driveToThirdDrop, driveToFourthIntake, AutoState.DRIVE_END)
                .addTransition(driveToThirdDrop, driveToFourthIntake, AutoState.TIMEOUT)

                .addTransition(driveToFourthIntake, intakeFourthSample, AutoState.DRIVE_END)
                .addTransition(driveToFourthIntake, intakeFourthSample, AutoState.TIMEOUT)

                .addTransition(intakeFourthSample, failedIntakeSubmersible, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(intakeFourthSample, failedIntakeSubmersible, AutoState.TIMEOUT)
                .addTransition(intakeFourthSample, driveToFourthDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(failedIntakeSubmersible, failedIntakeSubmersible, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(failedIntakeSubmersible, driveToFourthDrop, AutoState.SAMPLE_4_INTAKE_RECOVERED)
                .addTransition(failedIntakeSubmersible, driveToFourthDrop, AutoState.TIMEOUT)

                .addTransition(driveToFourthDrop, driveToFifthIntake, AutoState.DRIVE_END)
                .addTransition(driveToFourthDrop, driveToFifthIntake, AutoState.TIMEOUT)

                .addTransition(driveToFifthIntake, intakeFifthSample, AutoState.DRIVE_END)
                .addTransition(driveToFifthIntake, intakeFifthSample, AutoState.TIMEOUT)

                .addTransition(intakeFifthSample, failedIntakeSubmersible, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(intakeFifthSample, failedIntakeSubmersible, AutoState.TIMEOUT)
                .addTransition(intakeFifthSample, driveToFifthDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(failedIntakeSubmersible, driveToFifthDrop, AutoState.SAMPLE_5_INTAKE_RECOVERED)
                .addTransition(failedIntakeSubmersible, driveToFifthDrop, AutoState.TIMEOUT)

                .addTransition(driveToFifthDrop, driveToSubmersible, AutoState.DRIVE_END)
                .addTransition(driveToFifthDrop, driveToSubmersible, AutoState.TIMEOUT)

                .addTransition(driveToSubmersible, endState, AutoState.DRIVE_END)
                .addTransition(driveToSubmersible, endState, AutoState.TIMEOUT)

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
        AutoSubsystem auto = new AutoSubsystem(stateMachine, robotState);
        registerSubsystems(auto, drive, odometry, dropper, intake, sensor, limelight, visualDisplaySubsystem);

//        disableUpdate();
    }

    @Override
    public void update() {
        telemetry.addData("Current X", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Current Y", robotState.getRobotCurrentPose().getY());
        telemetry.addData("Current Heading", Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Current Auto State: ", robotState.getCurrentAutoState());
        telemetry.addData("Robot Block Position: ", robotState.getBlockPosition());
        telemetry.addLine();
        telemetry.addData("Expected X", robotState.getRobotFinalPose().getX());
        telemetry.addData("Expected Y", robotState.getRobotFinalPose().getY());
        telemetry.addData("Expected Heading", Math.toDegrees(robotState.getRobotFinalPose().getHeading()));
        telemetry.addData("Block Detection State", robotState.getCoarseBlockDetectionState());
        telemetry.addLine();
        telemetry.addData("block forward distance", robotState.getBlockForwardCoarse());
        telemetry.addData("block lateral distance", robotState.getBlockLateralCoarse());
        telemetry.addLine();
        double currentPos = intake.getCurrentSlidePositionInches();
        double expectedPos = intake.getTargetPositionInches();
        telemetry.addData("Current slide position (inches)", currentPos);
        telemetry.addData("Expected slide position (inches)", expectedPos);
        telemetry.addLine();
        telemetry.addData("Expected Dropper Slide Position", dropper.getTargetPositionInches());
        telemetry.addData("Dropper Slide Position", dropper.getCurrentSlidePositionInches());
    }

    @Override
    public void justAfterStart() {
        robotState.resetTimer();
    }

    @Override
    public void end() {
        RobotSaveState.getInstance().setState("robotCurrentPose", robotState.getRobotCurrentPose());
    }
}
