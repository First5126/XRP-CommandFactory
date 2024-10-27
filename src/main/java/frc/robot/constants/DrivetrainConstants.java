// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public final class DrivetrainConstants {
    /* Need to convert distance travelled to degrees. The Standard
       XRP Chassis found here, https://www.sparkfun.com/products/22230,
       has a wheel placement diameter (163 mm) - width of the wheel (8 mm) = 155 mm
       or 6.102 inches. We then take into consideration the width of the tires.
    */
    public static final double kWheeleBaseInches = 6.102;
    public static final double kInchesPerDegree = Math.PI * kWheeleBaseInches / 360;
    public static final double kGearRatio =
    (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
    public static final double kCountsPerMotorShaftRev = 12.0;
    public static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
    public static final double kWheelDiameterInch = 2.3622; // 60 mm

    public static class LineFollowingConstants {
        public static final double kTurnP = .75;
        public static final double kLineSpeed = 0.2;
    }
}
