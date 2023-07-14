
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DriverConstant;

public class DriveSubsystem extends SubsystemBase {
  /********** DRIVER MOTOR ***********************/
  private CANSparkMax m_left_front = new CANSparkMax(DriverConstant.left_front, MotorType.kBrushless);
  private CANSparkMax m_left_back = new CANSparkMax(DriverConstant.left_back, MotorType.kBrushless);
  private CANSparkMax m_right_front = new CANSparkMax(DriverConstant.right_front, MotorType.kBrushless);
  private CANSparkMax m_right_back = new CANSparkMax(DriverConstant.right_back, MotorType.kBrushless);
  /********** DIFFERENTIALDRIVE ******************/
  public DifferentialDrive m_driver = new DifferentialDrive(m_left_front, m_right_front);
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.5);
  /********** ENCODER ******************/
  RelativeEncoder m_leftEncoder = m_left_front.getEncoder();
  RelativeEncoder m_rightEncoder = m_right_front.getEncoder();

  /********** Auto Turn With PID ******************/
  PIDController m_DriverPidController = new PIDController(DriverConstant.kGains_GyroTurn.kP,DriverConstant.kGains_GyroTurn.kI, DriverConstant.kGains_GyroTurn.kD);
 
  /*****************GYRO**************************/
  Pigeon2 m_pigeon2 = new Pigeon2(DriverConstant.pigeno_ID, "rio");

  /******************************************************************
   * periodic
   *******************************************************************/
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Driver_LeftSpeed", m_left_front.getEncoder().getVelocity());
    SmartDashboard.putNumber("Driver_RightSpeed", m_right_front.getEncoder().getVelocity());
    SmartDashboard.putNumber("YAW", m_pigeon2.getYaw());
    
  }

  /******************************************************************
   * DriverSubsystem
   *******************************************************************/
  public DriveSubsystem() {
    m_left_front.setInverted(true);

    m_left_front.setIdleMode(IdleMode.kBrake);
    m_left_back.setIdleMode(IdleMode.kBrake);
    m_right_front.setIdleMode(IdleMode.kBrake);
    m_right_back.setIdleMode(IdleMode.kBrake);

    m_left_back.follow(m_left_front);
    m_right_back.follow(m_right_front);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    m_pigeon2.setYaw(0);
  }

  /******************************************************************
   * Drive
   *******************************************************************/
  public void TankDrive(double leftSpeed, double rightSpeed) {
    m_driver.tankDrive(leftSpeed, rightSpeed);
  }

  public void ArcadeDrive(double YSpeed, double XSpeed) {
    m_driver.arcadeDrive(YSpeed, -XSpeed);
  }

  public void TestDrive(double leftSpeed, double rightSpeed) {
    m_left_front.set(-leftSpeed);
    m_right_front.set(rightSpeed);
  }

  /***********************************************
   * Turn RPM To Meter per seconds
   ************************************************/
  public double get_drive_speed(){
    double speedL =  m_leftEncoder.getVelocity()* DriverConstant.Rpm2Mps;
    double speedR = m_rightEncoder.getVelocity()* DriverConstant.Rpm2Mps;
    double speed = (speedL+speedR)/2;
    return speed;
  }

  /**************************************************
   * contral Car go straight
   * <p> target_meter_left: TargetPosition of left Motor
   * <p> target_meter_right: TargetPosition of right Motor
   * <p> Example: go_straight(1.2, drive_current_drivestraight);
   * ? i dont know waht is 21.65 , mabey is Push Car go one meter and see what Enocder.getposition() changed.
   * <p> that my calculate result is 22.428
   *************************************************/
  public void AutoDrivesRun(double target_meter_left,double target_meter_right) {
    
    double current_encoder_left  = m_leftEncoder. getPosition();    //Unit:1 pre rotate circle
    double current_encoder_right  = m_rightEncoder. getPosition();

    double out_left = ((target_meter_left * 21.65) - current_encoder_left) * DriverConstant.DriverRun_kp;
    double out_right = ((target_meter_right * 21.65) - current_encoder_right) * DriverConstant.DriverRun_kp;

    m_driver.tankDrive(out_left, out_right);
  }
  /**************************************************
   * Turn with Gyro & PIDcontraller
   * <p> target 
   *************************************************/
  public void auto_turn( double target_angle){
    double auto_turn_output= m_DriverPidController.calculate(m_pigeon2.getYaw(), target_angle);
    m_driver.tankDrive(auto_turn_output , -auto_turn_output);
  }

}
