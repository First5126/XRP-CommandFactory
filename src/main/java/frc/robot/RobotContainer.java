// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.constants.DrivetrainConstants.LineFollowingConstants;
import frc.robot.sensors.LineSensor;
import frc.robot.sensors.UltrasonicSensor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.constants.ArmConstants.ArmPositions.kArmMinDegrees;
import static frc.robot.constants.DrivetrainConstants.LineFollowingConstants;

import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XRPOnBoardIO m_onboardIO = new XRPOnBoardIO();
  private final Arm m_arm = new Arm();
  private final UltrasonicSensor m_ultrasonicSensor = new UltrasonicSensor();
  // Assumes a gamepad plugged into channel 0
  private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final LineSensor m_lineSensor = new LineSensor();

  private AutonomousCommandFactory m_autonomousCommandFactory = new AutonomousCommandFactory();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutonomousCommands();
  }

  /**
   * Configures the button bindings for the robot.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(m_drivetrain.arcadeDriveCommand(() -> -m_controller.getLeftY(),() -> -m_controller.getRightX()));

    // Example of how to use the onboard IO
    Trigger userButton = new Trigger(m_onboardIO::getUserButtonPressed);
    userButton
        .onTrue(new PrintCommand("USER Button Pressed"))
        .onFalse(new PrintCommand("USER Button Released"));

    m_controller.a()
        .onTrue(m_arm.rotateToDegreesCommand(45))
        .onFalse(m_arm.rotateToDegreesCommand(kArmMinDegrees));

    m_controller.b()
        .onTrue(m_arm.rotateToDegreesCommand(90.0))
        .onFalse(m_arm.rotateToDegreesCommand(kArmMinDegrees));

    m_ultrasonicSensor.getCollisionTrigger()
        .onTrue(m_drivetrain.turnDegreesCommand(LineFollowingConstants.kLineSpeed, 90.0));

    m_controller.x().and(m_lineSensor.lineDetectedTrigger())
      .whileTrue(m_drivetrain.driveLine(.5,m_lineSensor::getDifference));
  }

  private void configureAutonomousCommands() {
       // Setup the SmartDashboard chooser for autonomous routines
    m_chooser.setDefaultOption("Auto Routine Distance",  m_autonomousCommandFactory.autonomousDistanceCommand(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", m_autonomousCommandFactory.autonomousTimeCommand(m_drivetrain));
    SmartDashboard.putData(m_chooser);  
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}
