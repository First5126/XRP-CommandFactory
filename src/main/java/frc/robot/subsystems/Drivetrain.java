// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.constants.DrivetrainConstants.kCountsPerRevolution;
import static frc.robot.constants.DrivetrainConstants.kInchesPerDegree;
import static frc.robot.constants.DrivetrainConstants.kWheelDiameterInch;
import static frc.robot.constants.HardwareIDs.MotorIDs.kLeftDriveMotorID;
import static frc.robot.constants.HardwareIDs.MotorIDs.kRightDriveMotorID;
import static frc.robot.constants.HardwareIDs.EncoderIDs.kLeftEncoderA;
import static frc.robot.constants.HardwareIDs.EncoderIDs.kLeftEncoderB;
import static frc.robot.constants.HardwareIDs.EncoderIDs.kRightEncoderA;
import static frc.robot.constants.HardwareIDs.EncoderIDs.kRightEncoderB;
import static frc.robot.constants.DrivetrainConstants.LineFollowingConstants;

public class Drivetrain extends SubsystemBase {


  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(kLeftDriveMotorID);
  private final XRPMotor m_rightMotor = new XRPMotor(kRightDriveMotorID);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(kLeftEncoderA, kLeftEncoderB);
  private final Encoder m_rightEncoder = new Encoder(kRightEncoderA, kRightEncoderB);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // For TurnTime Command
  private long m_startTime;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
  }

  private void arcadeDrive(Supplier<Double> xaxisSpeed, Supplier<Double> zaxisRotate) {
    arcadeDrive(xaxisSpeed.get(), zaxisRotate.get());
  }

  private void arcadeDrive(Double xaxisSpeed, Double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  private void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the XRP along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the XRP along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the XRP along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the XRP around the X-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the XRP around the Y-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the XRP around the Z-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  private void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getAverageTurningDistance() {
    double leftDistance = Math.abs(getLeftDistanceInch());
    double rightDistance = Math.abs(getRightDistanceInch());
    return (leftDistance + rightDistance) / 2.0;
  }

  public Command getArcadeDriveCommand(CommandXboxController controller) {
    Supplier<Double> driveVelocity = () -> -controller.getLeftY();
    Supplier<Double> rotateVelocity = () -> -controller.getRightX();
    return run(()->arcadeDrive(driveVelocity, rotateVelocity));
  }

  
  /**
   * Creates a command that turns the drivetrain at a specified speed for a certain duration of time.
   * 
   * @param speed The speed at which the drivetrain should turn. Positive values turn to the right, negative values turn to the left.
   * @param duration The duration of time (in milliseconds) for which the drivetrain should turn.
   * @return The command object that turns the drivetrain for the specified duration at the specified speed.
   */
  public Command turnTimeCommand(double speed, double duration){
    return new FunctionalCommand(
        () -> this.m_startTime = System.currentTimeMillis(), 
        () -> arcadeDrive( 0.0,  speed),
        (interrupted) ->  arcadeDrive( 0.0, 0.0),
        ()-> (System.currentTimeMillis() - m_startTime) >= duration,this);
  }

  /**
   * Creates a command that drives the robot at a specified speed for a specified duration of time.
   * 
   * @param speed The speed at which the robot should drive. Positive values indicate forward motion, while negative values indicate backward motion.
   * @param duration The duration of time, in milliseconds, for which the robot should drive at the specified speed.
   * @return The command object that drives the robot for the specified duration at the specified speed.
   */
  public Command driveTimeCommand(double speed, double duration){
    return new FunctionalCommand(
        () -> this.m_startTime = System.currentTimeMillis(), 
        () -> arcadeDrive( speed, 0.0),
        (interrupted) ->  arcadeDrive( 0.0, 0.0),
        ()-> (System.currentTimeMillis() - m_startTime) >= duration,this);
  }

  /**
   * Creates a command to drive a certain distance at a given speed.
   * 
   * @param speed The speed at which to drive (between -1.0 and 1.0).
   * @param distance The distance to drive in inches.
   * @return The command to drive the specified distance at the specified speed.
   */
  public Command driveDistanceCommand(double speed, double distance){
    return new FunctionalCommand(
        () -> resetEncoders(), 
        () -> arcadeDrive( speed, 0.0),
        (interrupted) ->  arcadeDrive( 0.0, 0.0),
        ()-> getAverageDistanceInch() >= distance,this);
  }

  /**
   * Creates a command to turn the drivetrain by a specified number of degrees at a given speed.
   * 
   * @param speed The speed at which the drivetrain should turn.
   * @param degrees The number of degrees to turn the drivetrain.
   * @return The command to turn the drivetrain.
   */
  public Command turnDegreesCommand(double speed, double degrees){
    return new FunctionalCommand(
        () -> resetEncoders(), 
        () -> arcadeDrive( 0.0, speed),
        (interrupted) ->  arcadeDrive( 0.0, 0.0),
        ()-> getAverageTurningDistance() >= kInchesPerDegree * degrees,this);
  }

  public Command driveLine(double speed, Supplier<Double> difference){
    return runEnd(
      ()->arcadeDrive(speed, LineFollowingConstants.kTurnP *  difference.get()),
      ()->arcadeDrive(0.0, 0.0)).alongWith(new PrintCommand("difference = " + difference.get() + " output = " + LineFollowingConstants.kTurnP * difference.get() ));
  }
}
