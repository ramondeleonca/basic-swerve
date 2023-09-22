// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SwerveDrive m_swerveDrive = new SwerveDrive();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
