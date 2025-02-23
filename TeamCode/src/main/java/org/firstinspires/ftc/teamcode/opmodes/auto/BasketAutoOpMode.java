package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.autostates.VisionSubmersiblePickupState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveFromSubmersibleSampleDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSampleDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSampleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSubmersibleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToPreloadDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToSubmersible;
import org.firstinspires.ftc.teamcode.autostates.basket.DropState;
import org.firstinspires.ftc.teamcode.autostates.basket.FirstLevelAscentState;
import org.firstinspires.ftc.teamcode.autostates.basket.IntakeSampleState;
import org.firstinspires.ftc.teamcode.autostates.basket.IntakeSampleWallState;
import org.firstinspires.ftc.teamcode.autostates.specimen.EndState;
import org.firstinspires.ftc.teamcode.display.view.TeleView;
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
        VisualDisplaySubsystem visualDisplaySubsystem = new VisualDisplaySubsystem(displayDriver, new TeleView(robotState));

        // Creating states
        DriveToPreloadDropState driveToPreloadDrop = new DriveToPreloadDropState(
                "driveToPreloadDrop",
                drive,
                dropper,
                intake,
                15,
                robotState);
        BasketDriveStateConfigurator.configPreloadDrop(driveToPreloadDrop);

        DropState dropSample = new DropState(
                "drop",
                dropper);

        DriveToGeneralSampleIntakeState driveToFirstIntake = new DriveToGeneralSampleIntakeState(
                "driveToFirstIntake",
                drive,
                dropper,
                intake,
                robotState);
        BasketDriveStateConfigurator.configFirstSampleIntake(driveToFirstIntake);

        IntakeSampleState intakeFirstSample = new IntakeSampleState(
                "intakeFirstSample",
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

        IntakeSampleState intakeSecondSample = new IntakeSampleState(
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
                17,
                robotState);
        BasketDriveStateConfigurator.configSecondSampleDrop(driveToSecondDrop);

        DriveToGeneralSampleIntakeState driveToThirdIntake = new DriveToGeneralSampleIntakeState(
                "driveToThirdIntake",
                drive,
                dropper,
                intake,
                robotState);
        BasketDriveStateConfigurator.configThirdSampleIntake(driveToThirdIntake);

        IntakeSampleWallState intakeThirdSample = new IntakeSampleWallState(
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

        VisionSubmersiblePickupState submersiblePickup = new VisionSubmersiblePickupState(
                "submersiblePickup",
                intake,
                dropper,
                drive,
                limelight,
                robotState
        );

        DriveToGeneralSubmersibleIntakeState driveToFourthIntake = new DriveToGeneralSubmersibleIntakeState(
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
                0,
                robotState
        );
        BasketDriveStateConfigurator.configFourthSampleDrop(driveToFourthDrop);

        DriveToGeneralSubmersibleIntakeState driveToFifthIntake = new DriveToGeneralSubmersibleIntakeState(
                "driveToFifthIntake",
                drive,
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

        FirstLevelAscentState firstLevelAscent = new FirstLevelAscentState(
                "firstLevelAscent",
                dropper,
                intake,
                robotState
        );

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
                .addState(submersiblePickup)
                .addState(driveToFourthIntake)
                .addState(driveToFourthDrop)
                .addState(driveToFifthIntake)
                .addState(driveToFifthDrop)
                .addState(driveToSubmersible)
                .addState(firstLevelAscent)
                .addState(endState)

                .addTransition(driveToPreloadDrop, driveToFirstIntake, AutoState.DRIVE_END)
                .addTransition(driveToPreloadDrop, driveToFirstIntake, AutoState.TIMEOUT)

                .addTransition(driveToFirstIntake, intakeFirstSample, AutoState.DRIVE_END)
                .addTransition(driveToFirstIntake, intakeFirstSample, AutoState.TIMEOUT)
                .addTransition(intakeFirstSample, driveToFirstDrop, AutoState.SAMPLE_INTAKE_COMPLETE)

                .addTransition(driveToFirstDrop, driveToSecondIntake, AutoState.DRIVE_END)
                .addTransition(driveToFirstDrop, driveToSecondIntake, AutoState.TIMEOUT)

                .addTransition(driveToSecondIntake, intakeSecondSample, AutoState.DRIVE_END)
                .addTransition(driveToSecondIntake, intakeSecondSample, AutoState.TIMEOUT)
                .addTransition(intakeSecondSample, driveToSecondDrop, AutoState.SAMPLE_INTAKE_COMPLETE)

                .addTransition(driveToSecondDrop, driveToThirdIntake, AutoState.DRIVE_END)
                .addTransition(driveToSecondDrop, driveToThirdIntake, AutoState.TIMEOUT)

                .addTransition(driveToThirdIntake, intakeThirdSample, AutoState.DRIVE_END)
                .addTransition(driveToThirdIntake, intakeThirdSample, AutoState.TIMEOUT)
                .addTransition(intakeThirdSample, driveToThirdDrop, AutoState.SAMPLE_INTAKE_COMPLETE)

                .addTransition(driveToThirdDrop, driveToFourthIntake, AutoState.DRIVE_END)
                .addTransition(driveToThirdDrop, driveToFourthIntake, AutoState.TIMEOUT)

                .addTransition(driveToFourthIntake, submersiblePickup, AutoState.DRIVE_END)
                .addTransition(driveToFourthIntake, submersiblePickup, AutoState.TIMEOUT)
                .addTransition(submersiblePickup, driveToFourthDrop, AutoState.SAMPLE_4_INTAKE_COMPLETE)
                .addTransition(submersiblePickup, driveToFourthDrop, AutoState.TIMEOUT)

                .addTransition(driveToFourthDrop, driveToSubmersible, AutoState.DRIVE_END)
                .addTransition(driveToFourthDrop, driveToSubmersible, AutoState.TIMEOUT)

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
        AutoSubsystem auto = new AutoSubsystem(stateMachine);
        registerSubsystems(auto, drive, odometry, dropper, intake, sensor, limelight, visualDisplaySubsystem);
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
