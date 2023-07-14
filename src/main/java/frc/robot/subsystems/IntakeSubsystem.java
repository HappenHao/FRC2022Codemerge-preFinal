
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax m_intake = new CANSparkMax(DeliverConstant.intake, MotorType.kBrushless);

  private Compressor m_compression = new Compressor(controlConstant.PCM, PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid m_intakeUD = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticConstant.k_IntakeD,PneumaticConstant.k_IntakeU);

  public IntakeSubsystem() {
    m_intake.setIdleMode(IdleMode.kCoast);
    m_intakeUD.set(DoubleSolenoid.Value.kOff);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Comperssion_ON", !m_compression.getPressureSwitchValue() );

    if(!m_compression.getPressureSwitchValue())
    {
      m_compression.enableDigital();
    }
    else{
      m_compression.disable();
    }
  }

  
  public void InakeUp() {
    m_intakeUD.set(PneumaticStatues.kIntakeUp);
  }

  public void  intakeDown() {
    m_intakeUD.set(PneumaticStatues.kIntakeDown);
  }

  public void stopMotor() {
    m_intake.stopMotor();
  }


  public void rotate(double clockValue, double unclockValue) {
    SmartDashboard.putNumber("IntakeSpeed_RPM", m_intake.getEncoder().getVelocity());

    if (clockValue > 0.1 && unclockValue < 0.1)
      m_intake.set(-clockValue);
    else if (clockValue < 0.1 && unclockValue > 0.1)
      m_intake.set(unclockValue);
    else
      m_intake.stopMotor();
  }

}
