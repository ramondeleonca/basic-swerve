// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(Constants.Swerve.kFrontLeftOptions);
  private final SwerveModule m_frontRight = new SwerveModule(Constants.Swerve.kFrontRightOptions);
  private final SwerveModule m_backLeft = new SwerveModule(Constants.Swerve.kBackLeftOptions);
  private final SwerveModule m_backRight = new SwerveModule(Constants.Swerve.kBackRightOptions);
  
  private final AHRS m_gyro = new AHRS(Port.kMXP);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.Swerve.PhysicalModel.kDriveKinematics, getRobotYawRotation2d(), getSwervePositions());

  public SwerveDrive() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        resetGyro();
      } catch (Exception err) {
        System.out.println("Error: " + err.getMessage() + "\n Can't reset gyro.");
      }
    }).start();
  }

  public void resetGyro() { m_gyro.reset(); }
  public double getRobotYaw() { return Math.IEEEremainder(m_gyro.getAngle(), 360.0); }
  public Rotation2d getRobotYawRotation2d() { return Rotation2d.fromDegrees(getRobotYaw()); }
  

  public SwerveModulePosition[] getSwervePositions () {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  public Pose2d getPose2d() { return m_odometry.getPoseMeters(); }
  public void resetOdometry(Pose2d pose) { m_odometry.resetPosition(getRobotYawRotation2d(), getSwervePositions(), pose); }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.PhysicalModel.kMaxSpeedMetersPerSecond);
    m_frontLeft.setState(desiredStates[0]);
    m_frontRight.setState(desiredStates[1]);
    m_backLeft.setState(desiredStates[2]);
    m_backRight.setState(desiredStates[3]);
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  @Override
  public void periodic() {
    m_odometry.update(getRobotYawRotation2d(), getSwervePositions());

    SmartDashboard.putNumber("Robot Angle", getRobotYaw());
    SmartDashboard.putString("Robot Location", getPose2d().getTranslation().toString());
    SmartDashboard.putString("Robot Rotation", getRobotYawRotation2d().toString());
  }
}
