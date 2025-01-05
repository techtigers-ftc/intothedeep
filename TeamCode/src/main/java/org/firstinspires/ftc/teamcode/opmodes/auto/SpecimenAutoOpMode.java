package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.autostates.specimen.DriveToPreloadDropStateSpecimen;
import org.firstinspires.ftc.teamcode.subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBodometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.BaseOpMode;
import team.techtigers.base.statemachine.StateMachine;
import team.techtigers.core.paths.Waypoint;

public class SpecimenAutoOpMode extends BaseOpMode {
    private RobotState robotState;

    @Override
    public void initialize() {
        StateMachine<AutoState> stateMachine = new StateMachine<>();
        robotState = new RobotState();

        // Initialize subsystems
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, robotState);
        GoBodometrySubsystem odometry = new GoBodometrySubsystem(hardwareMap,
                robotState, new Waypoint(77, 7.25, Math.toRadians(90)));
        DropperSubsystem dropper = new DropperSubsystem(hardwareMap,
                robotState);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, robotState);

        // Creating states
        DriveToPreloadDropStateSpecimen driveChamberPreload = new DriveToPreloadDropStateSpecimen(
                "driveToChamberPreload",
                drive,
                dropper,
                robotState);
        SpecimenStateConfigurator.configPreloadDrop(driveChamberPreload);



        stateMachine
                .addState(new DriveToPreloadDropStateSpecimen("driveToChamberPreload", drive, dropper, robotState))



                .setCurrentState(driveChamberPreload);




        // Register subsystems + Create state machine subsystem
        AutoSubsystem auto = new AutoSubsystem(stateMachine);
        registerSubsystems(auto, drive, odometry, dropper);
    }
}
