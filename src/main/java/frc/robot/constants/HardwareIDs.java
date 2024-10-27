// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

public final class HardwareIDs {
    
    /**
     * Contains the motor IDs used in the robot.
     */
    public static class MotorIDs {
        public static final int kLeftDriveMotorID = 0;
        public static final int kRightDriveMotorID = 1;
    }

    /**
     * Contains the encoder IDs used in the robot.
     */
    public static class EncoderIDs {
        public static final int kLeftEncoderA = 4;
        public static final int kLeftEncoderB = 5;
        public static final int kRightEncoderA = 6;
        public static final int kRightEncoderB = 7;
    }

    /**
     * Contains the analog input IDs used in the robot.
     */
    public static class AnalogInputIDs {
        public static final int kLeftReflectanceSensorID = 0;
        public static final int kRightReflectanceSensorID = 1;
        public static final int kUltrasonicSensorID = 2;
    }

    public static final int kGyroID = 0;
    public static final int kArmServoID = 4;
}
