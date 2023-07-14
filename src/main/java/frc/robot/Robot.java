package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.SensorConstant;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /********************* ADDRESS ABLE LED************************** */
  private AddressableLED m_led = new AddressableLED(SensorConstant.Addressable_LED);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(SensorConstant.Addressable_LED_Number);
  private double m_rainbowFirstPixelHue = 0;

  /***************************************************
   * ADDRESSABLE LED RAINBOW
   ******************************************************/
  private void rainbow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, (int) hue, 255, 128);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
  }

  /***************************************************
   ******************************************************/
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    /********************* ADDRESS ABLE LED************************** */
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    rainbow();
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /***********************************************************************************
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
  }

  /*******************************************************************************************
   * This autonomous runs the autonomous command selected by your
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {

  }

  /*******************************************************************************************
  
   */
  @Override
  public void teleopInit() {
    //
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //
    m_robotContainer.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleopPeriodic();
  }

  /****************************************************************************************
   * 
   */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {

  }
}
