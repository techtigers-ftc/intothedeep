package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.autostates.EndState;
import org.firstinspires.ftc.teamcode.autostates.specimen.ClipAndIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.ClipSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveFromChamberSampleDropState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstColoredSampleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstPushState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstSpecimenIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenDropState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToSpecimenPark;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPoseState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.IntakeAndDropColoredSampleState;
import org.firstinspires.ftc.teamcode.autostates.specimen.StrafeAndTransferState;
import org.firstinspires.ftc.teamcode.autostates.specimen.IntakeSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.TurnAndTransferState;
import org.firstinspires.ftc.teamcode.display.view.AutoView;
import org.firstinspires.ftc.teamcode.opmodes.auto.configurators.SpecimenDriveStateConfigurator;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.statemachine.StateMachine;
import team.techtigers.base.visualdisplay.AdafruitNeoPixel;
import team.techtigers.base.visualdisplay.VisualDisplaySubsystem;
import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.utils.RobotSaveState;

@Config
public abstract class SpecimenAutoOpMode extends BaseOpMode {
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
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap,
                robotState, new Waypoint(79.25, 7.25, Math.toRadians(90)));
        dropper = new DropperSubsystem(hardwareMap,
                robotState);
        intake = new IntakeSubsystem(hardwareMap, robotState);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState);
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);

        AdafruitNeoPixel displayDriver = hardwareMap.get(AdafruitNeoPixel.class, "visual_display");
        displayDriver.initialize(224, 3);
        VisualDisplaySubsystem visualDisplaySubsystem = new VisualDisplaySubsystem(displayDriver, new AutoView(robotState));

        // Sets color preference to alliance color
        robotState.setBlockColorPreference(BlockColorPreference.ALLIANCE);

        // Creating states
        DriveToPreloadDropSpecimenState driveToPreloadDrop = new DriveToPreloadDropSpecimenState(
                "driveToPreloadDrop",
                drive,
                intake,
                dropper,
                robotState);
        SpecimenDriveStateConfigurator.configPreloadDrop(driveToPreloadDrop);

        ClipAndIntakeState clipPreload = new ClipAndIntakeState(
                "clipPreload",
                drive,
                intake,
                dropper,
                robotState);

        ClipSpecimenState clipSpecimen = new ClipSpecimenState(
                "clipSpecimen",
                drive,
                dropper,
                robotState
        );

        IntakeSpecimenState intakeSpecimen = new IntakeSpecimenState(
                "intakeSpecimen",
                drive,
                dropper,
                robotState
        );

        DriveToFirstSpecimenIntakeState driveToFirstIntake = new DriveToFirstSpecimenIntakeState(
                "driveToFirstIntake",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configFirstIntake(driveToFirstIntake);

        DriveToFirstColoredSampleIntakeState driveToFirstSampleIntake = new DriveToFirstColoredSampleIntakeState(
                "driveToFirstSampleIntake",
                drive,
                intake,
                dropper,
                robotState,
                3
        );
        SpecimenDriveStateConfigurator.configFirstSampleIntake(driveToFirstSampleIntake);

        IntakeAndDropColoredSampleState intakeAndDropFirstSample = new IntakeAndDropColoredSampleState(
                "intakeAndDropFirstSample",
                drive,
                intake,
                dropper,
                () -> 16,
                robotState
        );
        StrafeAndTransferState strafeAndTransfer = new StrafeAndTransferState(
                "intakeAndTransfer",
                drive,
                intake,
                dropper,
                robotState,
                3
        );
        SpecimenDriveStateConfigurator.configSecondSampleIntake(strafeAndTransfer);

        IntakeAndDropColoredSampleState intakeAndDropSecondSample = new IntakeAndDropColoredSampleState(
                "intakeAndDropSecondSample",
                drive,
                intake,
                dropper,
                () -> 16,
                robotState
        );

        TurnAndTransferState turnAndTransfer = new TurnAndTransferState(
                "turnAndTransfer",
                drive,
                intake,
                dropper,
                robotState,
                3
        );
        SpecimenDriveStateConfigurator.configThirdSampleIntake(turnAndTransfer);

        IntakeAndDropColoredSampleState intakeAndDropThirdSample = new IntakeAndDropColoredSampleState(
                "intakeAndDropThirdSample",
                drive,
                intake,
                dropper,
                () -> 16,
                robotState
        );

        DriveToGeneralSpecimenDropState driveToFirstDrop = new DriveToGeneralSpecimenDropState(
                "driveToFirstDrop",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configFirstDrop(driveToFirstDrop);

        DriveToFirstPushState driveToFirstPush = new DriveToFirstPushState(
                "driveToFirstPush",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configDriveToFirstPush(driveToFirstPush);

        DriveToPoseState firstPush = new DriveToPoseState(
                "firstPush",
                drive,
                robotState,
                3
        );
        SpecimenDriveStateConfigurator.configFirstPush(firstPush);

        DriveToPoseState driveToSecondPush = new DriveToPoseState(
                "driveToSecondPush",
                drive,
                robotState
                , 3
        );
        SpecimenDriveStateConfigurator.configDriveToSecondPush(driveToSecondPush);

        DriveToPoseState secondPush = new DriveToPoseState(
                "secondPush",
                drive,
                robotState,
                3
        );
        SpecimenDriveStateConfigurator.configSecondPush(secondPush);

        DriveToPoseState driveToThirdPush = new DriveToPoseState(
                "driveToThirdPush",
                drive,
                robotState,
                3
        );
        SpecimenDriveStateConfigurator.configDriveToThirdPush(driveToThirdPush);

        DriveToPoseState thirdPush = new DriveToPoseState(
                "thirdPush",
                drive,
                robotState,
                3
        );
        SpecimenDriveStateConfigurator.configThirdPush(thirdPush);

        DriveToPoseState driveToSecondIntake = new DriveToPoseState(
                "driveToSecondIntake",
                drive,
                robotState,
                2
        );
        SpecimenDriveStateConfigurator.configSecondIntake(driveToSecondIntake);

        DriveToGeneralSpecimenDropState driveToSecondDrop = new DriveToGeneralSpecimenDropState(
                "driveToSecondDrop",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configSecondDrop(driveToSecondDrop);

        DriveToGeneralSpecimenIntakeState driveToThirdIntake = new DriveToGeneralSpecimenIntakeState(
                "driveToThirdIntake",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configGeneralIntake(driveToThirdIntake);

        DriveToGeneralSpecimenDropState driveToThirdDrop = new DriveToGeneralSpecimenDropState(
                "driveToThirdDrop",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configGeneralDrop(driveToThirdDrop);

        DriveToGeneralSpecimenIntakeState driveToFourthIntake = new DriveToGeneralSpecimenIntakeState(
                "driveToFourthIntake",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configGeneralIntake(driveToFourthIntake);

        DriveToGeneralSpecimenDropState driveToFourthDrop = new DriveToGeneralSpecimenDropState(
                "driveToFourthDrop",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configGeneralDrop(driveToFourthDrop);

        DriveToGeneralSpecimenIntakeState driveToFifthIntake = new DriveToGeneralSpecimenIntakeState(
                "driveToFifthIntake",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configGeneralIntake(driveToFifthIntake);

        DriveToGeneralSpecimenDropState driveToFifthDrop = new DriveToGeneralSpecimenDropState(
                "driveToFifthDrop",
                drive,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configGeneralDrop(driveToFifthDrop);

        DriveToSpecimenPark driveToPark = new DriveToSpecimenPark(
                "driveToPark",
                drive,
                intake,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configDriveToPark(driveToPark);

        DriveFromChamberSampleDropState driveToSampleDrop = new DriveFromChamberSampleDropState(
                "driveToSampleDrop",
                drive,
                intake,
                dropper,
                robotState
        );
        SpecimenDriveStateConfigurator.configSampleDrop(driveToSampleDrop);

        ClipAndIntakeState intakeSample = new ClipAndIntakeState(
                "intakeSample",
                drive,
                intake,
                dropper,
                robotState
        );

        EndState endState = new EndState("end");

        // Create the state machine
        stateMachine
                .addState(driveToPreloadDrop)
                .addState(clipPreload)
                .addState(clipSpecimen)
                .addState(intakeSpecimen)
                .addState(driveToFirstDrop)
                .addState(driveToSecondDrop)
                .addState(driveToFirstPush)
                .addState(driveToFirstIntake)
                .addState(driveToFirstSampleIntake)
                .addState(intakeAndDropFirstSample)
                .addState(strafeAndTransfer)
                .addState(intakeAndDropSecondSample)
                .addState(turnAndTransfer)
                .addState(intakeAndDropThirdSample)
                .addState(firstPush)
                .addState(driveToSecondPush)
                .addState(secondPush)
                .addState(driveToThirdPush)
                .addState(thirdPush)
                .addState(driveToSecondIntake)
                .addState(driveToSecondDrop)
                .addState(driveToThirdIntake)
                .addState(driveToThirdDrop)
                .addState(driveToFourthIntake)
                .addState(driveToFourthDrop)
                .addState(driveToFifthIntake)
                .addState(driveToFifthDrop)
                .addState(driveToPark)
                .addState(driveToSampleDrop)
                .addState(intakeSample)
                .addState(endState)

                // Drives to the preload and clips it
                .addTransition(driveToPreloadDrop, clipPreload, AutoState.DRIVE_END)
                .addTransition(driveToPreloadDrop, clipPreload, AutoState.TIMEOUT)
                // Drives to the first intake if a sample is intaken, goes to the first push if not
                .addTransition(clipPreload, driveToFirstSampleIntake, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(clipPreload, driveToFirstSampleIntake, AutoState.SAMPLE_INTAKE_FAILED)
                // Drives to the first intake once the preload is clipped
                .addTransition(driveToFirstSampleIntake, intakeAndDropFirstSample, AutoState.DRIVE_END)
                .addTransition(driveToFirstSampleIntake, intakeAndDropFirstSample, AutoState.TIMEOUT)

                .addTransition(intakeAndDropFirstSample, strafeAndTransfer, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(strafeAndTransfer, intakeAndDropSecondSample, AutoState.DRIVE_END)
                .addTransition(strafeAndTransfer, intakeAndDropSecondSample, AutoState.TIMEOUT)

                .addTransition(intakeAndDropSecondSample, turnAndTransfer, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(turnAndTransfer, intakeAndDropThirdSample, AutoState.DRIVE_END)
                .addTransition(turnAndTransfer, intakeAndDropThirdSample, AutoState.TIMEOUT)

                .addTransition(intakeAndDropThirdSample, endState, AutoState.SAMPLE_INTAKE_COMPLETE)




                // Intakes the first specimen
                .addTransition(driveToFirstIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFirstIntake, intakeSpecimen, AutoState.TIMEOUT)
                // Drives to drop the first specimen and clips it
                .addTransition(intakeSpecimen, driveToFirstDrop, AutoState.SPECIMEN_1_INTAKE_COMPLETE)
                .addTransition(driveToFirstDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFirstDrop, clipSpecimen, AutoState.TIMEOUT)
                // Goes to the first push once the first specimen has been clipped
                .addTransition(clipSpecimen, driveToFirstPush, AutoState.SPECIMEN_1_DROP_COMPLETE)
                // Pushes the first sample
                .addTransition(driveToFirstPush, firstPush, AutoState.DRIVE_END)
                .addTransition(driveToFirstPush, firstPush, AutoState.TIMEOUT)
                // Drives out to a position to push the second sample
                .addTransition(firstPush, driveToSecondPush, AutoState.DRIVE_END)
                .addTransition(firstPush, driveToSecondPush, AutoState.TIMEOUT)
                // Pushes the second sample
                .addTransition(driveToSecondPush, secondPush, AutoState.DRIVE_END)
                .addTransition(driveToSecondPush, secondPush, AutoState.TIMEOUT)
                // Drives out to a position to push the third sample
                .addTransition(secondPush, driveToThirdPush, AutoState.DRIVE_END)
                .addTransition(secondPush, driveToThirdPush, AutoState.TIMEOUT)
                // Drives to the third push
                .addTransition(driveToThirdPush, thirdPush, AutoState.DRIVE_END)
                .addTransition(driveToThirdPush, thirdPush, AutoState.TIMEOUT)
                // Drives to the second specimen intake
                .addTransition(thirdPush, driveToSecondIntake, AutoState.DRIVE_END)
                .addTransition(thirdPush, driveToSecondIntake, AutoState.TIMEOUT)
                // Intakes the second specimen
                .addTransition(driveToSecondIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToSecondIntake, intakeSpecimen, AutoState.TIMEOUT)
                // Drives to drop the second specimen and clips it
                .addTransition(intakeSpecimen, driveToSecondDrop, AutoState.SPECIMEN_2_INTAKE_COMPLETE)
                .addTransition(driveToSecondDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToSecondDrop, clipSpecimen, AutoState.TIMEOUT)
                // Drives to the third specimen intake
                .addTransition(clipSpecimen, driveToThirdIntake, AutoState.SPECIMEN_2_DROP_COMPLETE)
                // Intakes the third specimen
                .addTransition(driveToThirdIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToThirdIntake, intakeSpecimen, AutoState.TIMEOUT)
                // Drives to drop the third specimen and clips it
                .addTransition(intakeSpecimen, driveToThirdDrop, AutoState.SPECIMEN_3_INTAKE_COMPLETE)
                .addTransition(driveToThirdDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToThirdDrop, clipSpecimen, AutoState.TIMEOUT)
                // Drives to the fourth specimen intake
                .addTransition(clipSpecimen, driveToFourthIntake, AutoState.SPECIMEN_3_DROP_COMPLETE)
                // Intakes the fourth specimen
                .addTransition(driveToFourthIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFourthIntake, intakeSpecimen, AutoState.TIMEOUT)
                // Drives to drop the fourth specimen and clips it
                .addTransition(intakeSpecimen, driveToFourthDrop, AutoState.SPECIMEN_4_INTAKE_COMPLETE)
                .addTransition(driveToFourthDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFourthDrop, clipSpecimen, AutoState.TIMEOUT)
                // Drives to the fifth specimen intake
                .addTransition(clipSpecimen, driveToFifthIntake, AutoState.SPECIMEN_4_DROP_COMPLETE)
                // Intakes the fifth specimen
                .addTransition(driveToFifthIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFifthIntake, intakeSpecimen, AutoState.TIMEOUT)
                // Drives to drop the fifth specimen and clips it
                .addTransition(intakeSpecimen, driveToFifthDrop, AutoState.SPECIMEN_5_INTAKE_COMPLETE)
                .addTransition(driveToFifthDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFifthDrop, clipSpecimen, AutoState.TIMEOUT)
                // Drives to the park and transitions to the end state
                .addTransition(clipSpecimen, driveToPark, AutoState.SPECIMEN_5_DROP_COMPLETE)
                .addTransition(driveToPark, endState, AutoState.DRIVE_END)
                .addTransition(driveToPark, endState, AutoState.TIMEOUT)

                // OLD CODE FOR SAMPLE INTAKE + DROP
//                .addTransition(driveToFifthDrop, intakeSample, AutoState.DRIVE_END)
//                .addTransition(driveToFifthDrop, intakeSample, AutoState.TIMEOUT)
//
//                // Goes to park if the sample intake times out or fails
//                .addTransition(intakeSample, driveToPark, AutoState.SAMPLE_INTAKE_FAILED)
//                .addTransition(intakeSample, driveToPark, AutoState.TIMEOUT)
//                .addTransition(intakeSample, driveToPark, AutoState.NO_TIME)
//
//                //Transitions to end state when done with either park or sample drop drive

//                .addTransition(driveToSampleDrop, endState, AutoState.DRIVE_END)
//                .addTransition(driveToSampleDrop, endState, AutoState.TIMEOUT)

                .setCurrentState(driveToPreloadDrop);

        // Register subsystems + Create state machine subsystem
        AutoSubsystem auto = new AutoSubsystem(stateMachine, robotState);
        registerSubsystems(auto, drive, odometry, dropper, intake, limelight, sensor, visualDisplaySubsystem);
        telemetry.addData("Current X", robotState.getRobotCurrentPose().getX());
        telemetry.addData("Current Y", robotState.getRobotCurrentPose().getY());
        telemetry.addData("Current Heading", Math.toDegrees(robotState.getRobotCurrentPose().getHeading()));
        telemetry.addLine();
        telemetry.addData("Expected X", robotState.getRobotFinalPose().getX());
        telemetry.addData("Expected Y", robotState.getRobotFinalPose().getY());
        telemetry.addData("Expected Heading", Math.toDegrees(robotState.getRobotFinalPose().getHeading()));
        telemetry.addData("Expected Slide Position", intake.getTargetPositionInches());
        telemetry.addData("Slide Position", intake.getCurrentSlidePositionInches());
        telemetry.addLine();
        telemetry.addData("Expected Dropper Slide Position", dropper.getTargetPositionInches());
        telemetry.addData("Dropper Slide Position", dropper.getCurrentSlidePositionInches());
        telemetry.update();

        disableUpdate();
    }

    @Override
    public void justAfterStart() {
        robotState.resetTimer();
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
        telemetry.addData("Expected Intake Slide Position", intake.getTargetPositionInches());
        telemetry.addData("Intake Slide Position", intake.getCurrentSlidePositionInches());
        telemetry.addLine();
        telemetry.addData("Expected Dropper Slide Position", dropper.getTargetPositionInches());
        telemetry.addData("Dropper Slide Position", dropper.getCurrentSlidePositionInches());
    }

    @Override
    public void end() {
        RobotSaveState.getInstance().setState("robotCurrentPose", robotState.getRobotCurrentPose());
    }
}
