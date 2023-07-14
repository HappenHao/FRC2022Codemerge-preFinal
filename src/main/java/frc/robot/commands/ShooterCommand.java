
package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  
  ShooterSubsystem m_shooter;
  // DeliverSubsystem m_deliver;
  
  private final Supplier<Double> m_ShooterPowerFounction;
  private final Supplier<Boolean> m_autoTargetButtonFounction;


  public ShooterCommand(ShooterSubsystem p_shooter,Supplier<Double> p_ShooterPowerFounction,Supplier<Boolean> p_autoTargetButtonFounction) {
    m_ShooterPowerFounction =p_ShooterPowerFounction;
    m_shooter = p_shooter;
    m_autoTargetButtonFounction = p_autoTargetButtonFounction;
    // m_deliver = p_deliver;
    addRequirements(p_shooter);
    // addRequirements(p_deliver);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
   if (m_autoTargetButtonFounction.get())
   {
      m_shooter.auto_shoot();
   }
   else{
     
    double realTimeShootPower =(-m_ShooterPowerFounction.get() + 1 )/2;//get Joystick axis value
    double targetVelocity_RPM = realTimeShootPower * 5000.0;
    if(targetVelocity_RPM>500){
    m_shooter.setVelocity_RPM(targetVelocity_RPM);
    }
    else{
      m_shooter.stopShoot();
    }
  }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
