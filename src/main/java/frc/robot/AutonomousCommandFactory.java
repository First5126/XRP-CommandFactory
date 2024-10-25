// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/** 
 * This class is responsible for generating autonomous commands.
 */
public class AutonomousCommandFactory {
    /**
     * Generates a command for autonomous mode based on time.
     * 
     * @param drivetrain The drivetrain object used for driving commands.
     * @return The autonomous command based on time.
     */
    public Command autonomousTimeCommand(Drivetrain drivetrain){
        return drivetrain.driveTimeCommand(-0.6,2.0)
        .andThen(drivetrain.turnTimeCommand(-0.5,1.3))
        .andThen(drivetrain.driveTimeCommand(-0.6,2.0))
        .andThen(drivetrain.turnTimeCommand(0.5,1.3));
    }

    /**
     * Generates a command for autonomous mode based on distance.
     * 
     * @param drivetrain The drivetrain object used for driving commands.
     * @return The autonomous command based on distance.
     */
    public Command autonomousDistanceCommand(Drivetrain drivetrain){
        return drivetrain.driveDistanceCommand(-0.5,10)
        .andThen(drivetrain.turnDegreesCommand(-0.5,180))
        .andThen(drivetrain.driveDistanceCommand(-0.5,10))
        .andThen(drivetrain.turnDegreesCommand(0.5,180));
    }
}
