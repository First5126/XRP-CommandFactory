// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.UltrasonicConstants;

import static frc.robot.constants.HardwareIDs.AnalogInputIDs.kUltrasonicSensorID;

/** Add your docs here. */
public class UltrasonicSensor {
    private AnalogInput m_ultrasonicSensor = new AnalogInput(kUltrasonicSensorID);

    public UltrasonicSensor() {
        super();
    }
    /**
     * Get the distance from the ultrasonic sensor.
     *
     * @return the distance in inches
     */
    public double getDistanceMilimeters() {
        double distance = m_ultrasonicSensor.getVoltage() / UltrasonicConstants.kMaxVoltage * (UltrasonicConstants.kMaxDistanceMilimeters - UltrasonicConstants.kMinDistanceMilimeters) + UltrasonicConstants.kMinDistanceMilimeters;
        SmartDashboard.putNumber("Distance",distance);
        return distance;
    }

    public Trigger getCollisionTrigger() {
        return new Trigger(() -> getDistanceMilimeters() < UltrasonicConstants.kCollisionDistanceMilimeters);
    }
}
