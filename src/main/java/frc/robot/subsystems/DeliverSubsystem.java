
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeliverConstant;

public class DeliverSubsystem extends SubsystemBase {

  private CANSparkMax m_deliver_front = new CANSparkMax(DeliverConstant.front, MotorType.kBrushed);
  private CANSparkMax m_deliver_back = new CANSparkMax(DeliverConstant.back, MotorType.kBrushed);

  
  

  public DeliverSubsystem() {
    m_deliver_front.setIdleMode(IdleMode.kBrake);
    m_deliver_back.setIdleMode(IdleMode.kBrake);
    m_deliver_front.setInverted(true);
    m_deliver_back.follow(m_deliver_front);

  }

  @Override
  public void periodic() {
    
  }

  public void deliverrun(double clockwise, double unclockwise) {
    if (clockwise > 0.1 && unclockwise < 0.1) {
      m_deliver_front.set(clockwise*DeliverConstant.DeliverInvent);
    } else if (clockwise < 0.1 && unclockwise > 0.1) {
      m_deliver_front.set(-unclockwise*DeliverConstant.DeliverInvent);
    } else {
      m_deliver_front.stopMotor();
    }
  }

  public void set_deliver() {
    m_deliver_front.set(1*DeliverConstant.DeliverInvent);
  }

  public void stop_deliver() {
    m_deliver_front.stopMotor();
  }

}
