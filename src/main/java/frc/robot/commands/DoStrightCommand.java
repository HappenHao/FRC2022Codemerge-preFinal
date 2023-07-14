// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DoStrightCommand extends CommandBase {
  DriveSubsystem m_driver;
  private double m_left_distance;
  private double m_right_distance;
  
  public DoStrightCommand(double leftMeters,double rightMeters,DriveSubsystem p_driver) {
    m_left_distance=leftMeters;
    m_right_distance=rightMeters;
    m_driver=p_driver;
    addRequirements(m_driver);
  }

  @Override
  public void initialize() {
    m_driver.resetEncoders();
  }

  @Override
  public void execute() {
    m_driver.AutoDrivesRun(m_left_distance,m_right_distance);
  }

  @Override
  public void end(boolean interrupted) {
    m_driver.ArcadeDrive(0, 0);
  }

  public boolean isFinished() {
    return Math.abs(m_driver.getAverageEncoderDistance()) >= (m_left_distance+m_right_distance)/2;
  }
}
