

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.team3526.lib.SwerveOptions;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule extends SubsystemBase {
  private SwerveOptions m_options;

  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turningMotor;

  private RelativeEncoder m_driveMotorEncoder;
  private RelativeEncoder m_turningMotorEncoder;

  private CANCoder m_turningEncoder;

  private PIDController m_turningPIDController;

  private SwerveModuleState m_state;

  private double m_initialAngleRad;
  
  public SwerveModule(SwerveOptions options) {
    this.m_options = options;

    this.m_driveMotor = new CANSparkMax(options.getDriveMotorID(), MotorType.kBrushless);
    this.m_turningMotor = new CANSparkMax(options.getTurningMotorID(), MotorType.kBrushless);

    this.m_driveMotor.setInverted(options.getDriveMotorInverted());
    this.m_turningMotor.setInverted(options.getTurningMotorInverted());
    
    this.m_driveMotorEncoder = m_driveMotor.getEncoder();
    this.m_turningMotorEncoder = m_turningMotor.getEncoder();

    this.m_driveMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kDriveEncoder_RotationToMeter);
    this.m_driveMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kDriveEncoder_RPMToMeterPerSecond);
    this.m_turningMotorEncoder.setPositionConversionFactor(Constants.Swerve.Module.kTurningEncoder_RotationToRadian);
    this.m_turningMotorEncoder.setVelocityConversionFactor(Constants.Swerve.Module.kTurningEncoder_RPMToRadianPerSecond);

    this.m_turningPIDController = new PIDController(Constants.Swerve.Module.TurningPIDParameters.kP, Constants.Swerve.Module.TurningPIDParameters.kI, Constants.Swerve.Module.TurningPIDParameters.kD);
    this.m_turningPIDController.enableContinuousInput(Math.toRadians(0), Math.toRadians(360));

    this.m_turningEncoder = new CANCoder(options.getTurningEncoderID());

    this.m_state = new SwerveModuleState();

    // if (this.m_turningEncoder.configMagnetOffset(options.getTurningEncoderOffsetDeg()) != ErrorCode.OK)
    //   System.out.println("WARN: " + options.getName() + " turning encoder failed to set magnet offset to " + options.getTurningEncoderOffsetDeg() + " degrees");

    if (this.m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360) != ErrorCode.OK)
      System.out.println("WARN: " + options.getName() + " turning encoder failed to set absolute sensor range to Unsigned_0_to_360");

    if (this.m_turningEncoder.setPositionToAbsolute() != ErrorCode.OK)
      System.out.println("WARN: " + options.getName() + " turning encoder failed to set position to absolute");

    this.m_initialAngleRad = Math.toRadians(this.m_turningEncoder.getAbsolutePosition());

    resetMotorEncoders();
  }

  // Reset functions
  public void resetMotorEncoders() {
    this.m_driveMotorEncoder.setPosition(0);
    this.m_turningMotorEncoder.setPosition(0);
  }

  public void stop() {
    this.m_driveMotor.set(0);
    this.m_turningMotor.set(0);
  }

  public void calibrateMagnetOffset() {
    new Thread(() -> {
      try {
        if (this.m_turningEncoder.configMagnetOffset(0) != ErrorCode.OK) throw new Exception("Failed to resetset " + m_options.getName() + " encoder offset to 0");
        System.out.println("WARN: DO NOT TURN " + m_options.getName() + " WHEEL / TURNING MOTOR");
        Thread.sleep(100);
        if (this.m_turningEncoder.setPositionToAbsolute() != ErrorCode.OK) throw new Exception("Failed to set " + m_options.getName() + " encoder position to absolute");
        double offset = this.m_turningEncoder.getAbsolutePosition() * -1;
        if (this.m_turningEncoder.configMagnetOffset(offset) != ErrorCode.OK) throw new Exception("Failed to set " + m_options.getName() + " encoder offset to " + offset + " degrees");
        this.m_initialAngleRad = 0;
        System.out.println("INFO: " + m_options.getName() + " encoder offset set to " + offset + " degrees");
      } catch (Exception err) {
        System.out.println("ERROR: " + m_options.getName() + " turning encoder failed to calibrate magnet offset \n" + err.getMessage() + "\n" + err.getStackTrace());
      }
    }).run();
    
  }

  // Turning position encoder
  public double getAngleRad() {
    return (m_turningMotorEncoder.getPosition() * (m_options.getTurningEncoderInverted() ? -1 : 1)) - m_initialAngleRad;
  }

  // Set the optimized swerve state
  public void setState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    this.m_state = SwerveModuleState.optimize(state, new Rotation2d(getAngleRad()));
    m_driveMotor.set(this.getState().speedMetersPerSecond / Constants.Swerve.PhysicalModel.kMaxSpeedMetersPerSecond);
    m_turningMotor.set(m_turningPIDController.calculate(getAngleRad(), this.getState().angle.getRadians()));
  }

  // Get the current optimized swerve state
  public SwerveModuleState getState() { return this.m_state; }

  // Telemetry Data
  public double getDriveMotorPosition() { return m_driveMotorEncoder.getPosition(); }
  public double getTurningMotorPosition() { return m_turningMotorEncoder.getPosition(); }

  public double getDriveVelocity() { return m_driveMotorEncoder.getVelocity(); }
  public double getTurningVelocity() { return m_turningMotorEncoder.getVelocity(); }

  public SwerveModulePosition getPosition() { return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getAngleRad())); }

  @Override
  public void periodic() {
    // Update the telemetry data
    SmartDashboard.putString(m_options.getName()+" Module State", getState().toString());
    SmartDashboard.putString(m_options.getName()+" Module Position", getPosition().toString());
    SmartDashboard.putNumber(m_options.getName()+" Module Speed", getDriveVelocity());
    SmartDashboard.putNumber(m_options.getName()+" Module Turning Speed", getTurningVelocity());
    SmartDashboard.putNumber(m_options.getName()+" Drive Encoder", getDriveMotorPosition());
    SmartDashboard.putNumber(m_options.getName()+" Turning Encoder", getTurningMotorPosition());
    SmartDashboard.putNumber(m_options.getName()+" Encoder Degrees", Math.toDegrees(getAngleRad()));
    SmartDashboard.putNumber(m_options.getName()+" Encoder Radians", getAngleRad());
    SmartDashboard.putData(m_options.getName() + " turning PID", this.m_turningPIDController);
  }
}
