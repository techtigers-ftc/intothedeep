package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.autostates.specimen.ClipPreloadState;
import org.firstinspires.ftc.teamcode.autostates.specimen.ClipSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToFirstIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenDropState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToGeneralSpecimenIntakeState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPark;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPoseState;
import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropSpecimenState;
import org.firstinspires.ftc.teamcode.autostates.EndState;
import org.firstinspires.ftc.teamcode.autostates.specimen.PickupSpecimenState;
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

                // Drives to the preload and clips it
                .addTransition(driveChamberPreload, clipPreload, AutoState.DRIVE_END)
                .addTransition(driveChamberPreload, clipPreload, AutoState.TIMEOUT)
                // Drives to the first intake once the preload is clipped
                .addTransition(clipPreload, driveToFirstIntake, AutoState.SPECIMEN_PRELOAD_DROP_COMPLETE)
                // Pushes the first sample
                .addTransition(driveToFirstIntake, firstPush, AutoState.DRIVE_END)
                .addTransition(driveToFirstIntake, firstPush, AutoState.TIMEOUT)
                // Drives out to a position to push the second sample
                .addTransition(firstPush, secondIntake, AutoState.DRIVE_END)
                .addTransition(firstPush, secondIntake, AutoState.TIMEOUT)
                // Pushes the second sample
                .addTransition(secondIntake, secondPush, AutoState.DRIVE_END)
                .addTransition(secondIntake, secondPush, AutoState.TIMEOUT)
                // Drives out to a position to push the third sample
                .addTransition(secondPush, thirdIntake, AutoState.DRIVE_END)
                .addTransition(secondPush, thirdIntake, AutoState.TIMEOUT)
                // Drives to the third push
                .addTransition(thirdIntake, thirdPush, AutoState.DRIVE_END)
                .addTransition(thirdIntake, thirdPush, AutoState.TIMEOUT)
                // Drives to the first specimen intake
                .addTransition(thirdPush, driveToFirstSpecimenIntake, AutoState.DRIVE_END)
                .addTransition(thirdPush, driveToFirstSpecimenIntake, AutoState.TIMEOUT)
                // Intakes the first specimen
                .addTransition(driveToFirstSpecimenIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFirstSpecimenIntake, intakeSpecimen, AutoState.TIMEOUT)
                // Drives to drop the first specimen and clips it
                .addTransition(intakeSpecimen, driveToFirstSpecimenDrop, AutoState.SPECIMEN_1_INTAKE_COMPLETE)
                .addTransition(driveToFirstSpecimenDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFirstSpecimenDrop, clipSpecimen, AutoState.TIMEOUT)
                // Drives to the second specimen intake
                .addTransition(clipSpecimen, driveToSecondSpecimenIntake, AutoState.SPECIMEN_1_DROP_COMPLETE)
                // Intakes the second specimen
                .addTransition(driveToSecondSpecimenIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToSecondSpecimenIntake, intakeSpecimen, AutoState.TIMEOUT)
                // Drives to drop the second specimen and clips it
                .addTransition(intakeSpecimen, driveToSecondSpecimenDrop, AutoState.SPECIMEN_2_INTAKE_COMPLETE)
                .addTransition(driveToSecondSpecimenDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToSecondSpecimenDrop, clipSpecimen, AutoState.TIMEOUT)
                // Drives to the third specimen intake
                .addTransition(clipSpecimen, driveToThirdSpecimenIntake, AutoState.SPECIMEN_2_DROP_COMPLETE)
                // Intakes the third specimen
                .addTransition(driveToThirdSpecimenIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToThirdSpecimenIntake, intakeSpecimen, AutoState.TIMEOUT)
                // Drives to drop the third specimen and clips it
                .addTransition(intakeSpecimen, driveToThirdSpecimenDrop, AutoState.SPECIMEN_3_INTAKE_COMPLETE)
                .addTransition(driveToThirdSpecimenDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToThirdSpecimenDrop, clipSpecimen, AutoState.TIMEOUT)
                // Drives to the fourth specimen intake
                .addTransition(clipSpecimen, driveToFourthSpecimenIntake, AutoState.SPECIMEN_3_DROP_COMPLETE)
                // Intakes the fourth specimen
                .addTransition(driveToFourthSpecimenIntake, intakeSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFourthSpecimenIntake, intakeSpecimen, AutoState.TIMEOUT)
                // Drives to drop the fourth specimen and clips it
                .addTransition(intakeSpecimen, driveToFourthSpecimenDrop, AutoState.SPECIMEN_4_INTAKE_COMPLETE)
                .addTransition(driveToFourthSpecimenDrop, clipSpecimen, AutoState.DRIVE_END)
                .addTransition(driveToFourthSpecimenDrop, clipSpecimen, AutoState.TIMEOUT)
                // Drives to the park in the observation zone
                .addTransition(clipSpecimen, driveToPark, AutoState.SPECIMEN_4_DROP_COMPLETE)
                .addTransition(driveToPark, endState, AutoState.DRIVE_END)
                .addTransition(driveToPark, endState, AutoState.TIMEOUT)

                .setCurrentState(driveChamberPreload);


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

//        disableUpdate();
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
