// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swerve.DriveSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final SwerveModule m_frontLeft = m_swerveDrive.getFrontLeft();
  private final SwerveModule m_frontRight = m_swerveDrive.getFrontRight();
  private final SwerveModule m_backLeft = m_swerveDrive.getBackLeft();
  private final SwerveModule m_backRight = m_swerveDrive.getBackRight();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("Calibrate Front Left Encoder Magnet", new InstantCommand(() -> m_frontLeft.calibrateMagnetOffset(), m_frontLeft));
    SmartDashboard.putData("Calibrate Front Right Encoder Magnet", new InstantCommand(() -> m_frontRight.calibrateMagnetOffset(), m_frontRight));
    SmartDashboard.putData("Calibrate Back Left Encoder Magnet", new InstantCommand(() -> m_backLeft.calibrateMagnetOffset(), m_backLeft));
    SmartDashboard.putData("Calibrate Front Right Encoder Magnet", new InstantCommand(() -> m_backRight.calibrateMagnetOffset(), m_backRight));

    m_swerveDrive.setDefaultCommand(new DriveSwerve(m_swerveDrive, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightX(), () -> !m_driverController.a().getAsBoolean()));

    configureBindings();
  }

  /** Configure the controller bindings to the triggers */
  private void configureBindings() {
    m_driverController.rightTrigger(Constants.OperatorConstants.kDeadZone).whileTrue(new DriveSwerve(m_swerveDrive, () -> 0.0, () -> m_driverController.getRightTriggerAxis() * -1, () -> 0.0, () -> !m_driverController.a().getAsBoolean()));
  }

  /** Return the command used for the autonomous period */
  public Command getAutonomousCommand() {
    return null;
  }
}
