// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.UltrasonicConstants;

import static frc.robot.constants.HardwareIDs.AnalogInputIDs.kUltrasonicSensorID;

/**
 * Represents an ultrasonic sensor used to measure distance.
 */
public class UltrasonicSensor {
    private AnalogInput m_ultrasonicSensor = new AnalogInput(kUltrasonicSensorID);
    private double m_RawSensorValue;

    /**
     * Constructs a new UltrasonicSensor object.
     * Registers the sensor for periodic reading of inputs.
     */
    public UltrasonicSensor() {
        super();
        // Add the readInputs method to the periodic update
        Robot.getInstance().addPeriodic(this::readInputs,TimedRobot.kDefaultPeriod);
    }

    /**
     * Reads the raw sensor value and updates the internal variable.
     */
    public void readInputs(){
       m_RawSensorValue = m_ultrasonicSensor.getVoltage();
    }

    /**
     * Get the distance from the ultrasonic sensor.
     *
     * @return the distance in millimeters
     */
    public double getDistanceMilimeters() {
        double distance = m_RawSensorValue / UltrasonicConstants.kMaxVoltage * (UltrasonicConstants.kMaxDistanceMilimeters - UltrasonicConstants.kMinDistanceMilimeters) + UltrasonicConstants.kMinDistanceMilimeters;
        return distance;
    }

    /**
     * Get a trigger that activates when the distance measured by the ultrasonic sensor
     * is less than the collision distance.
     *
     * @return the collision trigger
     */
    public Trigger getCollisionTrigger() {
        return new Trigger(() -> getDistanceMilimeters() < UltrasonicConstants.kCollisionDistanceMilimeters);
    }
}
