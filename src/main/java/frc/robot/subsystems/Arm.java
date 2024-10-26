// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.HardwareIDs.kArmServoID;

public class Arm extends SubsystemBase {
  private final XRPServo m_armServo;

  /** Creates a new Arm. */
  public Arm() {
    // Device number 4 maps to the physical Servo 1 port on the XRP
    m_armServo = new XRPServo(kArmServoID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Set the current angle of the arm (0 - 180 degrees).
   *
   * @param angleDeg Desired arm angle in degrees
   */
  private void setAngle(double angleDeg) {
    // Ensure the angle is within the valid range
    if (angleDeg >= 0 && angleDeg <=180) {
      m_armServo.setAngle(angleDeg);
    }
  }

  /**
   * Sets the angle of the arm in degrees.
   * 
   * @param angleDeg the desired angle in degrees
   * @return the Command object that sets the angle
   */
  public Command setAngleDegreesCommand(double angleDeg) {
    return runOnce(()->setAngle(angleDeg));
  }
}
