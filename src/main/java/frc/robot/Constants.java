// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int JOYSTICK_PORT = 0;
    public static final int XBOX_CONTROLLER_PORT = 1;

    public static final int J_SLOW_MODE_BUTTON = 2;
  }

  public static class DriveConstants {
    // Motor Can ID's (Configured with Phoenix Tuner)
    public static final int kLEFT_FRONT_CAN_ID = 10;
    public static final int kLEFT_REAR_CAN_ID = 20;
    public static final int kRIGHT_FRONT_CAN_ID = 30;
    public static final int kRIGHT_REAR_CAN_ID = 40;

    // SysID Gains (Unknown as of right now)
    public static final double ksVolts = 0.11455;
    public static final double kvVoltSecondsPerMeter = 2.2002;
    public static final double kaVoltSecondsSquaredPerMeter = 0.3135;
    // Acutal Value from Sysid : 0.79737
    public static final double kPDriveVel = 0.8;

    // ! Unknown Value (Only needed for simulation)
    public static final double kDriveTrainMomentOfInertia = 4.0;
    public static final double kTrackWidth = Units.inchesToMeters(23.014);
    public static final DifferentialDriveKinematics kKinematics = new DifferentialDriveKinematics(kTrackWidth);

    public static final double kMaxAccelerationMetersPerSecondSq = 4.0;
    public static final double kMaxVelocityMetersPerSecons = 4.0;

    // Ramsete Controller (Values Recommended by WPILib)
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // 2023 Competition Bot Gear Ratio
    public static final double kGearRatio = 9.76;
    public static final double kWheelRadiusInches = 3;

    public static final double kLinearDistanceConversionFactor = (Units
        .inchesToMeters(1 / (kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches))) * 10d);

  }

  public static class PortsConstants {
    /* Servo Port */
    public static final int GRABBER_PORT = 0;

    // Arm motor ports
    public static final int LINEAR_ACTUATOR_PORT = 14;
    public static final int EXTENTION_PORT = 15;
  }
}
