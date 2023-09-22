// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.team3526.lib.SwerveOptions;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadZone = 0.1;
  }

  public static class Swerve {
    public static final class Module {
      public static final double kWheelDiameterMeters = 0.1016; // 4 inches
      public static final double kDriveMotorGearRatio = 1.0 / 6.12; // 6.12:1 Drive
      public static final double kTurningMotorGearRatio = 1.0 / 12.8; // 12.8:1 Steering

      public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI; // Conversion Rotaciones a Metros
      public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0; // Conversion RPM a Metros por Segundo

      public static final double kTurningEncoder_RotationToRadian = kTurningMotorGearRatio * 2.0 * Math.PI; // Conversion Rotaciones a Radianes
      public static final double kTurningEncoder_RPMToRadianPerSecond = kTurningEncoder_RotationToRadian / 60.0; // Conversion RPM a Radianes por Segundo

      public static final class TurningPIDParameters {
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
    }

    public static final class PhysicalModel {
      public static final double kTrackWidth = 0.5842; // Distance between right and left wheels
      public static final double kWheelBase = 0.5842; // Distance between front and back wheels

      // Sweerve Kinematics
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
      );

      // Speeds
      public static final double kMaxSpeedMetersPerSecond = 5.0; // Maxima Velocidad en Metros por Segundo
      public static final double kMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // Maxima Velocidad Angular en Radianes por Segundo

      public static final double kMaxAccelerationUnitsPerSecond = 3; // Maxima Aceleracion
      public static final double kMaxAngularAccelerationUnitsPerSecond = Math.PI / 4; // Maxima Aceleracion Angular

      public static final double kTeleopMaxSpeedMetersPerSecond = kMaxSpeedMetersPerSecond / 4; // Maxima Velocidad en Metros por Segundo
      public static final double kTeleopMaxAngularSpeedRadiansPerSecond = kMaxAngularSpeedRadiansPerSecond / 4; // Maxima Velocidad Angular en Radianes por Segundo

      public static final double kTeleopMaxAccelerationUnitsPerSecond = 3; // Maxima Aceleracion
      public static final double kTeleopMaxAngularAccelerationUnitsPerSecond = 3; // Maxima Aceleracion Angular

      // Trapezoid Profile Constraints
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(kTeleopMaxAngularSpeedRadiansPerSecond, kTeleopMaxAngularAccelerationUnitsPerSecond);
    }

    public static final SwerveOptions kFrontLeftOptions = new SwerveOptions(
      0.0,
      false,
      2,
      6,
      7,
      true,
      true,
      "Front Left"
    );
    public static final SwerveOptions kFrontRightOptions = new SwerveOptions(
      0.0,
      false,
      4,
      10,
      11,
      true,
      true,
      "Front Right"
    );

    public static final SwerveOptions kBackLeftOptions = new SwerveOptions(
      0.0,
      false,
      3,
      8,
      9,
      false,
      true,
      "Back Left"
    );
    public static final SwerveOptions kBackRightOptions = new SwerveOptions(
      -0.038568841727186,
      false,
      5,
      12,
      13,
      false,
      true,
      "Back Right"
    );
  }
}
