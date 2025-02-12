// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends Command {
  /** Creates a new SwerveDrive. */
  Drivetrain _driveTrain;
  public SwerveDrive(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drivetrain.swerveDrive(
      -RobotContainer.controller.getLeftY(), 
      -RobotContainer.controller.getLeftX(), 
      -RobotContainer.controller.getRightX(), 
      !RobotContainer.controller.getRawButton(XboxController.Button.kA.value),
      true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
