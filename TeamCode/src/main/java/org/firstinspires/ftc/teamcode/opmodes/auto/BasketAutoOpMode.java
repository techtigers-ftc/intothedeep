package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.autostates.basket.DriveFromSubmersibleSampleDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSampleDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSampleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToGeneralSubmersibleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToPreloadDropState;
import org.firstinspires.ftc.teamcode.autostates.basket.DriveToSamplePark;
import org.firstinspires.ftc.teamcode.autostates.basket.FailedIntakeSampleState;
import org.firstinspires.ftc.teamcode.autostates.basket.FailedSubmersibleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.FirstLevelAscentState;
import org.firstinspires.ftc.teamcode.autostates.basket.SampleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.basket.SubmersibleIntakeState;
import org.firstinspires.ftc.teamcode.display.view.AutoView;
import org.firstinspires.ftc.teamcode.opmodes.auto.configurators.BasketDriveStateConfigurator;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

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

//        AdafruitNeoPixel displayDriver = hardwareMap.get(AdafruitNeoPixel.class, "visual_display");
//        displayDriver.initialize(224, 3);
//        VisualDisplaySubsystem visualDisplaySubsystem = new VisualDisplaySubsystem(displayDriver, new AutoView(robotState));

        // Creating states
        DriveToPreloadDropState driveToPreloadDrop = new DriveToPreloadDropState(
                "driveToPreloadDrop",
                drive,
                dropper,
                intake,
                12,
                robotState);
        BasketDriveStateConfigurator.configPreloadDrop(driveToPreloadDrop);

        DriveToGeneralSampleIntakeState driveToFirstIntake = new DriveToGeneralSampleIntakeState(
                "driveToFirstIntake",
                drive,
                dropper,
                robotState);
        BasketDriveStateConfigurator.configFirstSampleIntake(driveToFirstIntake);

        SampleIntakeState intakeFirstSample = new SampleIntakeState(
                "intakeFirstSample",
                drive,
                intake,
                robotState
        );

        FailedIntakeSampleState failedIntakeSample = new FailedIntakeSampleState(
                "failedIntakeSample",
                drive,
                intake,
                limelight,
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
                robotState);
        BasketDriveStateConfigurator.configSecondSampleIntake(driveToSecondIntake);

        SampleIntakeState intakeSecondSample = new SampleIntakeState(
                "intakeSecondSample",
                drive,
                intake,
                robotState
        );

        DriveToGeneralSampleDropState driveToSecondDrop = new DriveToGeneralSampleDropState(
                "driveToSecondDrop",
                drive,
                dropper,
                intake,
                15,
                robotState);
        BasketDriveStateConfigurator.configSecondSampleDrop(driveToSecondDrop);

        DriveToGeneralSampleIntakeState driveToThirdIntake = new DriveToGeneralSampleIntakeState(
                "driveToThirdIntake",
                drive,
                dropper,
                robotState);
        BasketDriveStateConfigurator.configThirdSampleIntake(driveToThirdIntake);

        SampleIntakeState intakeThirdSample = new SampleIntakeState(
                "intakeThirdSample",
                drive,
                intake,
                robotState
        );

        DriveToGeneralSampleDropState driveToThirdDrop = new DriveToGeneralSampleDropState(
                "driveToThirdDrop",
                drive,
                dropper,
                intake,
                1,
                robotState);
        BasketDriveStateConfigurator.configThirdSampleDrop(driveToThirdDrop);

        SubmersibleIntakeState intakeFourthSample = new SubmersibleIntakeState(
                "intakeFourthSample",
                drive,
                intake,
                limelight,
                robotState
        );

        FailedSubmersibleIntakeState failedIntakeSubmersible = new FailedSubmersibleIntakeState(
                "failedIntakeSubmersible",
                drive,
                intake,
                limelight,
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
                limelight,
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

        SubmersibleIntakeState intakeSixthSample = new SubmersibleIntakeState(
                "intakeSixthSample",
                drive,
                intake,
                limelight,
                robotState
        );

        DriveToGeneralSubmersibleIntakeState driveToSixthIntake = new DriveToGeneralSubmersibleIntakeState(
                "driveToSixthIntake",
                drive,
                intake,
                dropper,
                robotState
        );
        BasketDriveStateConfigurator.configSixthSampleIntake(driveToSixthIntake);

        DriveFromSubmersibleSampleDropState driveToSixthDrop = new DriveFromSubmersibleSampleDropState(
                "driveToSixthDrop",
                drive,
                dropper,
                intake,
                robotState
        );
        BasketDriveStateConfigurator.configSixthSampleDrop(driveToSixthDrop);

        DriveToSamplePark driveToPark = new DriveToSamplePark(
                "driveToPark",
                drive,
                dropper,
                intake,
                robotState
        );
        BasketDriveStateConfigurator.configDriveToPark(driveToPark);

        FirstLevelAscentState firstLevelAscent = new FirstLevelAscentState(
                "firstLevelAscent",
                drive,
                intake,
                dropper,
                robotState
        );

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
                .addState(intakeSixthSample)
                .addState(driveToSixthIntake)
                .addState(driveToSixthDrop)
                .addState(driveToPark)
                .addState(firstLevelAscent)

                // Drives to the preload, drops it off, and goes to the first intake
                .addTransition(driveToPreloadDrop, driveToFirstIntake, AutoState.DRIVE_END)
                .addTransition(driveToPreloadDrop, driveToFirstIntake, AutoState.TIMEOUT)

                // Intakes the first sample
                .addTransition(driveToFirstIntake, intakeFirstSample, AutoState.DRIVE_END)
                .addTransition(driveToFirstIntake, intakeFirstSample, AutoState.TIMEOUT)

                // Transitions from the first sample intake to the drop, including transitions if the intake fails
                .addTransition(intakeFirstSample, failedIntakeSample, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(intakeFirstSample, failedIntakeSample, AutoState.TIMEOUT)
                .addTransition(intakeFirstSample, driveToFirstDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(failedIntakeSample, failedIntakeSample, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(failedIntakeSample, driveToFirstDrop, AutoState.SAMPLE_1_INTAKE_RECOVERED)
                .addTransition(failedIntakeSample, driveToFirstDrop, AutoState.FAILED_SAMPLE_1_TIMEOUT)
//                .addTransition(driveToFirstDrop, driveToPark, AutoState.PARK)

                // Drives to the first drop, drops it off, and goes to the second intake
                .addTransition(driveToFirstDrop, driveToSecondIntake, AutoState.DRIVE_END)
                .addTransition(driveToFirstDrop, driveToSecondIntake, AutoState.TIMEOUT)

                // Intakes the second sample
                .addTransition(driveToSecondIntake, intakeSecondSample, AutoState.DRIVE_END)
                .addTransition(driveToSecondIntake, intakeSecondSample, AutoState.TIMEOUT)

                // Transitions from the second sample intake to the drop, including transitions if the intake fails
                .addTransition(intakeSecondSample, failedIntakeSample, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(intakeSecondSample, failedIntakeSample, AutoState.TIMEOUT)
                .addTransition(intakeSecondSample, driveToSecondDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(failedIntakeSample, driveToSecondDrop, AutoState.SAMPLE_2_INTAKE_RECOVERED)
                .addTransition(failedIntakeSample, driveToSecondDrop, AutoState.FAILED_SAMPLE_2_TIMEOUT)
//                .addTransition(driveToSecondDrop, driveToPark, AutoState.PARK)

                // Drives to the second drop, drops it off, and goes to the third intake
                .addTransition(driveToSecondDrop, driveToThirdIntake, AutoState.DRIVE_END)
                .addTransition(driveToSecondDrop, driveToThirdIntake, AutoState.TIMEOUT)

                // Intakes the third sample
                .addTransition(driveToThirdIntake, intakeThirdSample, AutoState.DRIVE_END)
                .addTransition(driveToThirdIntake, intakeThirdSample, AutoState.TIMEOUT)

                // Transitions from the third sample intake to the drop, including transitions if the intake fails
                .addTransition(intakeThirdSample, failedIntakeSample, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(intakeThirdSample, failedIntakeSample, AutoState.TIMEOUT)
                .addTransition(intakeThirdSample, driveToThirdDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(failedIntakeSample, driveToThirdDrop, AutoState.SAMPLE_3_INTAKE_RECOVERED)
                .addTransition(failedIntakeSample, driveToThirdDrop, AutoState.FAILED_SAMPLE_3_TIMEOUT)

                // Drives to the third drop, drops it off, and goes to the fourth intake
//                .addTransition(driveToThirdDrop, driveToPark, AutoState.PARK)
                .addTransition(driveToThirdDrop, driveToFourthIntake, AutoState.DRIVE_END)
                .addTransition(driveToThirdDrop, driveToFourthIntake, AutoState.TIMEOUT)

                // Intakes the fourth sample
                .addTransition(driveToFourthIntake, intakeFourthSample, AutoState.DRIVE_END)
                .addTransition(driveToFourthIntake, intakeFourthSample, AutoState.TIMEOUT)

                // Transitions from the fourth sample intake to the drop, including transitions if the intake fails
                .addTransition(intakeFourthSample, failedIntakeSubmersible, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(intakeFourthSample, failedIntakeSubmersible, AutoState.TIMEOUT)
                .addTransition(intakeFourthSample, driveToFourthDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
//                .addTransition(intakeFourthSample, firstLevelAscent, AutoState.PARK)
//                .addTransition(failedIntakeSubmersible, firstLevelAscent, AutoState.PARK)
                .addTransition(failedIntakeSubmersible, failedIntakeSubmersible, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(failedIntakeSubmersible, driveToFourthDrop, AutoState.SAMPLE_4_INTAKE_RECOVERED)
                .addTransition(failedIntakeSubmersible, failedIntakeSubmersible, AutoState.FAILED_SAMPLE_4_TIMEOUT)
//                .addTransition(driveToFourthDrop, driveToPark, AutoState.PARK)

                // Drives to the fourth drop, drops it off, and goes to the fifth intake
                .addTransition(driveToFourthDrop, driveToFifthIntake, AutoState.DRIVE_END)
                .addTransition(driveToFourthDrop, driveToFifthIntake, AutoState.TIMEOUT)

                // Intakes the fifth sample
                .addTransition(driveToFifthIntake, intakeFifthSample, AutoState.DRIVE_END)
                .addTransition(driveToFifthIntake, intakeFifthSample, AutoState.TIMEOUT)

                // Transitions from the fifth sample intake to the drop, including transitions if the intake fails
                .addTransition(intakeFifthSample, failedIntakeSubmersible, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(intakeFifthSample, failedIntakeSubmersible, AutoState.TIMEOUT)
                .addTransition(intakeFifthSample, driveToFifthDrop, AutoState.SAMPLE_INTAKE_COMPLETE)
//                .addTransition(intakeFifthSample, driveToPark, AutoState.PARK)
//                .addTransition(intakeFifthSample, firstLevelAscent, AutoState.PARK)
                .addTransition(failedIntakeSubmersible, driveToFifthDrop, AutoState.SAMPLE_5_INTAKE_RECOVERED)
//                .addTransition(driveToFifthDrop, driveToPark, AutoState.PARK)
//                .addTransition(driveToFifthDrop, driveToPark, AutoState.DRIVE_END)
//                .addTransition(driveToFifthDrop, driveToPark, AutoState.TIMEOUT)

                // Drives to the fourth drop, drops it off, and goes to the fifth intake
                .addTransition(driveToFifthDrop, driveToSixthIntake, AutoState.DRIVE_END)
                .addTransition(driveToFifthDrop, driveToSixthIntake, AutoState.TIMEOUT)

                // Intakes the fifth sample
                .addTransition(driveToSixthIntake, intakeSixthSample, AutoState.DRIVE_END)
                .addTransition(driveToSixthIntake, intakeSixthSample, AutoState.TIMEOUT)

                // Transitions from the fifth sample intake to the drop, including transitions if the intake fails
                .addTransition(intakeSixthSample, failedIntakeSubmersible, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(intakeSixthSample, failedIntakeSubmersible, AutoState.TIMEOUT)
                .addTransition(intakeSixthSample, driveToSixthDrop, AutoState.SAMPLE_INTAKE_COMPLETE)

                .addTransition(driveToSixthDrop, driveToPark, AutoState.DRIVE_END)
                .addTransition(driveToSixthDrop, driveToPark, AutoState.TIMEOUT)

                .addTransition(driveToPark, firstLevelAscent, AutoState.DRIVE_END)
                .addTransition(driveToPark, firstLevelAscent, AutoState.TIMEOUT)

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
//        registerSubsystems(limelight, auto, drive, odometry, dropper, intake, sensor, visualDisplaySubsystem);

        registerSubsystems(limelight, auto, drive, odometry, dropper, intake, sensor);

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
        telemetry.addData("Block Forward Fine: ", robotState.getBlockForwardFine());
        telemetry.addData("Block Lateral Fine: ", robotState.getBlockLateralFine());
        telemetry.addLine();
        telemetry.addLine();
        double currentPos = intake.getCurrentSlidePositionInches();
        double expectedPos = intake.getTargetPositionInches();
        telemetry.addData("Current slide position (inches)", currentPos);
        telemetry.addData("Expected slide position (inches)", expectedPos);
        telemetry.addLine();
        telemetry.addData("Expected Dropper Slide Position", dropper.getTargetPositionInches());
        telemetry.addData("Dropper Slide Position", dropper.getCurrentSlidePositionInches());
        telemetry.addLine();

        if (robotState.isBlockDetected()) {
            telemetry.addLine("Block Detected");
            telemetry.addData("Last Remembered Block Position", robotState.getAbsoluteBlockPosition());
        } else {
            telemetry.addLine("Block NOT Detected");
        }
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
