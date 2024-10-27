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

/** Add your docs here. */
public class LineSensor {
    private AnalogInput m_leftReflectanceSensor = new AnalogInput(kLeftReflectanceSensorID);
    private AnalogInput m_rightReflectanceSensor = new AnalogInput(kRightReflectanceSensorID);
    private double m_leftValue;
    private double m_rightValue;
    private double m_difference;

    public LineSensor() {
        super();
        Robot.getInstance().addPeriodic(this::readInputs,TimedRobot.kDefaultPeriod);
    }


    public void readInputs(){
        m_leftValue = m_leftReflectanceSensor.getVoltage();
        m_rightValue = m_rightReflectanceSensor.getVoltage();
        m_difference = m_leftValue - m_rightValue;

        // Output values to SmartDashboard
        SmartDashboard.putNumber("Left Reflectance Sensor Value", m_leftValue);
        SmartDashboard.putNumber("Right Reflectance Sensor Value", m_rightValue);
        SmartDashboard.putNumber("Difference", m_difference);
    }

    public double getLeftValue(){
        return m_leftValue;
    }

    public double getRightValue(){
        return m_rightValue;
    }

    public double getDifference(){
        return m_difference;
    }

    public boolean isOnLine(){
        return getRightValue() > LineSensorConstants.kDetectionThreshold || getLeftValue() > LineSensorConstants.kDetectionThreshold;
    }

    public Trigger lineDetectedTrigger(){
        return new Trigger(() -> isOnLine());
    }
}
