// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriverAutoTurnCommand extends CommandBase {
  DriveSubsystem m_driver;
  private double m_Angle;
  public DriverAutoTurnCommand(double TurnAngle,DriveSubsystem p_driver) {
    m_Angle=TurnAngle;
    m_driver=p_driver;
    addRequirements(m_driver);
  }

  @Override
  public void initialize() {
    m_driver. resetGyro();
  }

  @Override
  public void execute() {
    m_driver.auto_turn(m_Angle);
  }

  @Override
  public void end(boolean interrupted) {
    m_driver.ArcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_Angle-m_driver.getGyroYaw())<5;
  }
}
