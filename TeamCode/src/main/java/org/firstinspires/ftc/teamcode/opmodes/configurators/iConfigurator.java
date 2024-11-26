package org.firstinspires.ftc.teamcode.opmodes.configurators;

import org.firstinspires.ftc.teamcode.commands.PedroPathFollowCommand;

/**
 * Interface used to represent a configurator for the auto commands in order to set PIDs and other
 * parameters in commands based on the alliance and randomization
 */
public interface iConfigurator {
    void configTestDrive(PedroPathFollowCommand command);
}
