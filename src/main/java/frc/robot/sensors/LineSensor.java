// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.LineSensorConstants;

import static frc.robot.constants.HardwareIDs.AnalogInputIDs.kLeftReflectanceSensorID;
import static frc.robot.constants.HardwareIDs.AnalogInputIDs.kRightReflectanceSensorID;

/**
 * The LineSensor class represents a line sensor used in a robot.
 * It provides methods to read sensor inputs, retrieve sensor values,
 * and determine if the sensor is detecting a line.
 */
public class LineSensor {
    private AnalogInput m_leftReflectanceSensor = new AnalogInput(kLeftReflectanceSensorID);
    private AnalogInput m_rightReflectanceSensor = new AnalogInput(kRightReflectanceSensorID);
    private double m_leftValue;
    private double m_rightValue;
    private double m_difference;

    /**
     * Constructs a LineSensor object and adds it to the robot's periodic update.
     */
    public LineSensor() {
        super();
        // Add the readInputs method to the periodic update
        Robot.getInstance().addPeriodic(this::readInputs,TimedRobot.kDefaultPeriod);
    }

    /**
     * Reads the sensor inputs and updates the sensor values.
     */
    public void readInputs(){
        m_leftValue = m_leftReflectanceSensor.getVoltage();
        m_rightValue = m_rightReflectanceSensor.getVoltage();
        m_difference = m_leftValue - m_rightValue;

        // Output values to SmartDashboard
        SmartDashboard.putNumber("Left Reflectance Sensor Value", m_leftValue);
        SmartDashboard.putNumber("Right Reflectance Sensor Value", m_rightValue);
        SmartDashboard.putNumber("Difference", m_difference);
    }

    /**
     * Returns the value of the left reflectance sensor.
     * 
     * @return The value of the left reflectance sensor.
     */
    public double getLeftValue(){
        return m_leftValue;
    }

    /**
     * Returns the value of the right reflectance sensor.
     * 
     * @return The value of the right reflectance sensor.
     */
    public double getRightValue(){
        return m_rightValue;
    }

    /**
     * Returns the difference between the left and right sensor values.
     * 
     * @return The difference between the left and right sensor values.
     */
    public double getDifference(){
        return m_difference;
    }

    /**
     * Determines if the line sensor is detecting a line.
     * 
     * @return True if the line sensor is detecting a line, false otherwise.
     */
    public boolean isOnLine(){
        return getRightValue() > LineSensorConstants.kDetectionThreshold || getLeftValue() > LineSensorConstants.kDetectionThreshold;
    }

    /**
     * Creates a trigger that activates when the line sensor detects a line.
     * 
     * @return A trigger that activates when the line sensor detects a line.
     */
    public Trigger lineDetectedTrigger(){
        return new Trigger(() -> isOnLine());
    }
}
