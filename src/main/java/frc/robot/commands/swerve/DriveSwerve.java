

package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class DriveSwerve extends CommandBase {
  SwerveDrive m_swerveDrive;
  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> rot;
  Supplier<Boolean> fieldRelative;

  SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(Constants.Swerve.PhysicalModel.kMaxAccelerationUnitsPerSecond);
  SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(Constants.Swerve.PhysicalModel.kMaxAccelerationUnitsPerSecond);
  SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.Swerve.PhysicalModel.kMaxAngularAccelerationUnitsPerSecond);

  public DriveSwerve(SwerveDrive swerveDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rot, Supplier<Boolean> fieldRelative) {
    this.m_swerveDrive = swerveDrive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rot = rot;
    this.fieldRelative = fieldRelative;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = this.xSpeed.get();
    double ySpeed = this.ySpeed.get();
    double rotSpeed = this.rot.get();

    xSpeed = Math.abs(xSpeed) > Constants.OperatorConstants.kDeadZone ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.OperatorConstants.kDeadZone ? ySpeed : 0.0;
    rotSpeed = Math.abs(rotSpeed) > Constants.OperatorConstants.kDeadZone ? rotSpeed : 0.0;

    xSpeed = xSpeedLimiter.calculate(xSpeed) * Constants.Swerve.PhysicalModel.kTeleopMaxSpeedMetersPerSecond;
    ySpeed = ySpeedLimiter.calculate(ySpeed) * Constants.Swerve.PhysicalModel.kTeleopMaxSpeedMetersPerSecond;
    rotSpeed = rotLimiter.calculate(rotSpeed) * Constants.Swerve.PhysicalModel.kTeleopMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds m_chassisSpeeds;
    if (fieldRelative.get()) {
      m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, m_swerveDrive.getRobotYawRotation2d());
    } else {
      m_chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    }

    SwerveModuleState[] m_moduleStates = Constants.Swerve.PhysicalModel.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds);
    m_swerveDrive.setModuleStates(m_moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
