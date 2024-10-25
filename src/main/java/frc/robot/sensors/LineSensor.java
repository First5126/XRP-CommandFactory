// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import static frc.robot.constants.HardwareIDs.AnalogInputIDs.kLeftReflectanceSensorID;

/** Add your docs here. */
public class LineSensor {
    private AnalogInput m_leftReflectanceSensor = new AnalogInput(kLeftReflectanceSensorID);
    private AnalogInput m_rightReflectanceSensor = new AnalogInput(kLeftReflectanceSensorID);

    public LineSensor() {
        super();
    }
}
