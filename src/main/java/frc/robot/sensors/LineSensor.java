// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Robot;

import static frc.robot.constants.HardwareIDs.AnalogInputIDs.kLeftReflectanceSensorID;

/** Add your docs here. */
public class LineSensor {
    private AnalogInput m_leftReflectanceSensor = new AnalogInput(kLeftReflectanceSensorID);
    private AnalogInput m_rightReflectanceSensor = new AnalogInput(kLeftReflectanceSensorID);
    private double m_leftValue;
    private double m_rightValue;

    public LineSensor() {
        super();
        Robot.getInstance().addPeriodic(this::readInputs,TimedRobot.kDefaultPeriod);
    }

    public void readInputs(){
        m_leftValue = m_leftReflectanceSensor.getVoltage();
        m_rightValue = m_rightReflectanceSensor.getVoltage();
    }

    public double getLeftValue(){
        return m_leftValue;
    }

    public double getRightValue(){
        return m_rightValue;
    }
}
