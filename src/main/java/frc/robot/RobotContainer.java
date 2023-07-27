// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.DuckSwerveDrive;

public class RobotContainer {
  private final CommandPS4Controller driverJoystick = new CommandPS4Controller(Constants.DriveControllerCharacteristics.port);

  private final DuckSwerveDrive swerveSubsystem = new DuckSwerveDrive();

  public RobotContainer() {
    
    swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem, driverJoystick::getLeftX, driverJoystick::getRightY, driverJoystick::getRightX));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
