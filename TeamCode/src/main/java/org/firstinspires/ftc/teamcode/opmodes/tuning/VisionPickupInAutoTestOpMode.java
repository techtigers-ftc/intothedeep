package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autostates.basket.SubmersibleIntakeState;
import org.firstinspires.ftc.teamcode.autostates.visiontest.DropperWallIntakeAndDropState;
import org.firstinspires.ftc.teamcode.autostates.visiontest.ReadyToTransferState;
import org.firstinspires.ftc.teamcode.autostates.visiontest.TrackAndIntakeState;
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
import team.techtigers.core.paths.Waypoint;

@Autonomous(name = "Vision Pickup In Auto Test", group = "Tuning")
public class VisionPickupInAutoTestOpMode extends BaseOpMode {
    private IntakeSubsystem intake;
    private DropperSubsystem dropper;
    private RobotState robotState;
    @Override
    public void initialize() {
        // Initializing and registering subsystems, state machine
        StateMachine<AutoState> stateMachine = new StateMachine<>();

        robotState = new RobotState(false, true);
        dropper = new DropperSubsystem(hardwareMap, robotState);
        intake = new IntakeSubsystem(hardwareMap, robotState);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap,
                robotState, new Waypoint(29.75, 7.25, Math.toRadians(90)));
        SensorSubsystem sensor = new SensorSubsystem(hardwareMap, robotState);
        AutoSubsystem auto = new AutoSubsystem(stateMachine, robotState);

        registerSubsystems(limelight, auto, drive, odometry, dropper, intake, sensor);

        // Creating the states
        TrackAndIntakeState trackAndIntakeState =
                new TrackAndIntakeState("Track and Intake", drive, intake, dropper, limelight, robotState);

        ReadyToTransferState goToReadyToTransfer =
                new ReadyToTransferState("Ready to Transfer", intake, dropper, robotState);

        DropperWallIntakeAndDropState dropperWallIntakeAndDropState =
                new DropperWallIntakeAndDropState("Dropper Wall Intake and Drop", intake, dropper, robotState);

        stateMachine
                .addState(trackAndIntakeState)
                .addState(goToReadyToTransfer)
                .addState(dropperWallIntakeAndDropState)

                .addTransition(trackAndIntakeState, goToReadyToTransfer, AutoState.SAMPLE_INTAKE_COMPLETE)
                .addTransition(trackAndIntakeState, trackAndIntakeState, AutoState.TIMEOUT)
                .addTransition(trackAndIntakeState, trackAndIntakeState, AutoState.SAMPLE_INTAKE_FAILED)
                .addTransition(goToReadyToTransfer, dropperWallIntakeAndDropState, AutoState.DRIVE_END)
                .addTransition(dropperWallIntakeAndDropState, trackAndIntakeState, AutoState.DRIVE_END)

                .setCurrentState(trackAndIntakeState);
    }

    public void update() {
        telemetry.addData("Intake State: ", robotState.getIntakeState());
        telemetry.addData("Dropper State: ", robotState.getDropperState());
        telemetry.addData("Is Block Detected? ", robotState.isBlockDetected());
        telemetry.addData("Current Auto State: ", robotState.getCurrentAutoState());
    }
}
