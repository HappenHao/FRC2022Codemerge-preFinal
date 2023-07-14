// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
  /***** MOTOR **************************/
  public WPI_TalonFX m_shooter_left_falcon = new WPI_TalonFX(ShooterConstant.left_shooter);
  public WPI_TalonFX m_shooter_right_falcon = new WPI_TalonFX(ShooterConstant.right_shooter);

  public CANSparkMax m_launch_motor = new CANSparkMax(ShooterConstant.launch, MotorType.kBrushless);
  public CANSparkMax m_rotate_motor = new CANSparkMax(ShooterConstant.rotate, MotorType.kBrushless);
  public CANSparkMax m_elevation_motor = new CANSparkMax(ShooterConstant.elevation, MotorType.kBrushless);

  /** LIMIT SWITCH ***********************/
  DigitalInput m_switch_elevation = new DigitalInput(SensorConstant.pitch_digital);
  DigitalInput m_switch_rotation = new DigitalInput(SensorConstant.rotate_digital);
  DigitalInput m_switch_tunnel = new DigitalInput(SensorConstant.shootExit_digital);

  /*** Encoder ***************************/
  RelativeEncoder m_rotate_encoder = m_rotate_motor.getEncoder();
  RelativeEncoder m_elevate_encoder = m_elevation_motor.getEncoder();

  /**** COLOR SENSOR **********************/
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

  /**** StringBuilder **********************/
  StringBuilder m_sb = new StringBuilder();

  /*** PHOTON VISION ********************/
  PhotonCamera m_camera = new PhotonCamera(PhotonVisionConstant.Shooter_Cam);

  PIDController m_rotateVersionpid = new PIDController(PhotonVisionConstant.kGains_rotateVersion.kP,
      PhotonVisionConstant.kGains_rotateVersion.kI, PhotonVisionConstant.kGains_rotateVersion.kD);

  /**** SHUFFLEBOARD & NETWORKTABLE ***********/
  ShuffleboardTab ShooterTab = Shuffleboard.getTab("ShooterTab");

  NetworkTableEntry ShooterSpeed = ShooterTab.add("Shooter Speed", 0).withWidget(BuiltInWidgets.kGraph)
      .withProperties(Map.of("min", 0, "max", 10000))
      .withSize(3, 3)
      .getEntry();

  public NetworkTableEntry TargetSpeed = ShooterTab.add("TargetSpeed", 0).getEntry();
  public NetworkTableEntry RotateEncoder = ShooterTab.add("RotateEncoder", 0).getEntry();
  public NetworkTableEntry ElevateEncoder = ShooterTab.add("ElevateEncoder", 0).getEntry();

  /*** PID CONTRALLER ********************/
  PIDController m_SpinPidController = new PIDController(ShooterConstant.kGains_Spin.kP, ShooterConstant.kGains_Spin.kI,
      ShooterConstant.kGains_Spin.kD);
  PIDController m_PitchPidController = new PIDController(ShooterConstant.kGains_Pitch.kP,
      ShooterConstant.kGains_Pitch.kI, ShooterConstant.kGains_Pitch.kD);

  // DriveSubsystem m_driver = new DriveSubsystem();

  /******************************************************************
   * ShooterSubsystem
   *******************************************************************/
  public ShooterSubsystem() {

    m_shooter_left_falcon.configFactoryDefault();
    m_shooter_right_falcon.configFactoryDefault();

    m_shooter_left_falcon.setNeutralMode(NeutralMode.Coast);
    m_shooter_right_falcon.setNeutralMode(NeutralMode.Coast);

    m_shooter_left_falcon.configNeutralDeadband(0.001);
    m_shooter_right_falcon.configNeutralDeadband(0.001);

    m_shooter_left_falcon.setInverted(TalonFXInvertType.Clockwise);
    m_shooter_right_falcon.setInverted(TalonFXInvertType.CounterClockwise);

    m_shooter_right_falcon.follow(m_shooter_left_falcon);

    m_launch_motor.setIdleMode(IdleMode.kBrake);
    m_rotate_motor.setIdleMode(IdleMode.kBrake);
    m_elevation_motor.setIdleMode(IdleMode.kBrake);

    m_rotate_encoder.setPosition(ShooterConstant.rotate_init_angle);
    m_elevate_encoder.setPosition(ShooterConstant.elevation_init_angle);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);

    /***** SHOOTER_PID ********************************************/
    m_shooter_left_falcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        ShooterConstant.kPIDLoopIdx, ShooterConstant.kTimeoutMs);

    m_shooter_left_falcon.config_kF(ShooterConstant.kPIDLoopIdx, ShooterConstant.kGains_Velocit.kF,
        ShooterConstant.kTimeoutMs);
    m_shooter_left_falcon.config_kP(ShooterConstant.kPIDLoopIdx, ShooterConstant.kGains_Velocit.kP,
        ShooterConstant.kTimeoutMs);
    m_shooter_left_falcon.config_kI(ShooterConstant.kPIDLoopIdx, ShooterConstant.kGains_Velocit.kI,
        ShooterConstant.kTimeoutMs);
    m_shooter_left_falcon.config_kD(ShooterConstant.kPIDLoopIdx, ShooterConstant.kGains_Velocit.kD,
        ShooterConstant.kTimeoutMs);

    /***** AUTO TURN TO ZERO WHEN CAR POWER ON ***********************/
    m_elevation_motor.set(0.3);
    m_rotate_motor.set(0.3);
  }

  /******************************************************************
   * periodic
   * 
   *******************************************************************/
  @Override
  public void periodic() {

    ShooterSpeed.setDouble(RawSensorUnittoRPM(m_shooter_left_falcon.getSelectedSensorVelocity()));
    ElevateEncoder.setDouble(m_elevate_encoder.getPosition());
    RotateEncoder.setDouble(m_rotate_encoder.getPosition());

    SmartDashboard.putBoolean("elevation_Limit", m_switch_elevation.get());
    SmartDashboard.putBoolean("Rotation_Limit", m_switch_rotation.get());
    SmartDashboard.putBoolean("Lounch_Switch", m_switch_tunnel.get());

    SmartDashboard.putNumber("Shooter_Speed_RPM", m_shooter_left_falcon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Flywheel_Velocity_mps",
        m_shooter_left_falcon.getSelectedSensorVelocity() * ShooterConstant.flywheel_up10ms_mps);

    resteEncoder();
    // getcolor();
    Ball_Stack();
  }

  /******************************************************************
   * RESTE Encoder of rotation and elevation
   *******************************************************************/
  private void resteEncoder() {
    if (!m_switch_rotation.get()) {
      m_rotate_encoder.setPosition(0);
    }
    if (m_switch_elevation.get()) {
      m_elevate_encoder.setPosition(0);
    }
    if (m_rotate_encoder.getPosition() > ShooterConstant.rotate_left_limit
        | m_rotate_encoder.getPosition() < ShooterConstant.rotate_right_limit) {
      m_rotate_motor.stopMotor();
    }
    if (m_elevate_encoder.getPosition() > ShooterConstant.elevation_limit | m_elevate_encoder.getPosition() < 0) {
      m_elevation_motor.stopMotor();
    }
  }

  /******************************************************************
   * judged team is blue or red
   *******************************************************************/
  public String IsBlue(double setteam) {
    String team = "null";
    if (setteam < 0) {
      team = "blue";
      SmartDashboard.putBoolean("teamset", true);
    } else {
      team = "red";
      SmartDashboard.putBoolean("teamset", false);
    }
    return team;
  }

  /******************************************************************
   * GetColor
   *******************************************************************/
  public String getcolor() {
    String color = "";
    Color detectedColor = m_colorSensor.getColor();
    double R = detectedColor.red;
    double G = detectedColor.green;
    double B = detectedColor.blue;

    if ((Math.abs(0.15 - R) < 0.1) && (Math.abs(0.4 - G) < 0.1) && (Math.abs(0.44 - B) < 0.1)) {
      color = "blue";
    } else if ((Math.abs(0.53 - R) < 0.2) && (Math.abs(0.34 - G) < 0.2) && (Math.abs(0.118 - B) < 0.2)) {
      color = "red";
    } else if ((Math.abs(0.25 - R) < 0.1) && (Math.abs(0.49 - G) < 0.1) && (Math.abs(0.26 - B) < 0.1)) {
      color = "empty";
    } else {
      color = "empty";
    }
    return color;
  }

  /******************************************************************
   * Ball_Stack
   *******************************************************************/
  public String ball_1 = " ";
  public String ball_2 = " ";

  public void Ball_Stack() {
    if (ball_1 == "empty") {
      ball_1 = getcolor();
    } else if (!(ball_1 == getcolor())) {
      ball_2 = getcolor();
    }
    if (m_switch_tunnel.get()) {
      ball_1 = ball_2;
    }
  }

  /******************************************************************
   * If Team is red but collecct a blue ball , return 20
   *******************************************************************/
  public double color_select(String team, String color) {
    double color_spin = 0;
    if (team != color) {
      color_spin = 20;
    } else {
      color_spin = 0;
    }
    return color_spin;
  }

  /******************************************************************
   * ROTATE
   *******************************************************************/
  public void rotate_clock() {
    m_rotate_motor.set(0.3);
  }

  public void rotate_unclock() {
    m_rotate_motor.set(-0.3);
  }

  public void rotate_stop() {
    m_rotate_motor.stopMotor();
  }

  /******************************************************************
   * ELEVATION
   *******************************************************************/
  public void elevation_clock() {
    m_elevation_motor.set(0.2);
  }

  public void elevation_unclock() {
    m_elevation_motor.set(-0.2);
  }

  public void elevation_stop() {
    m_elevation_motor.stopMotor();
  }

  /******************************************************************
   * LAUNCH
   *******************************************************************/
  public void launch() {
    m_launch_motor.set(-1);
  }

  public void stopLaunch() {
    m_launch_motor.stopMotor();
  }

  /******************************************************************
   * FLYWHEEL
   ********************************************************************/
  public void setPower(double power) {
    m_shooter_left_falcon.set(ControlMode.PercentOutput, power);
  }

  public void setVelocity_RPM(double velocity) {
    TargetSpeed.setDouble(velocity);
    if (velocity > 500) {
      m_shooter_left_falcon.set(ControlMode.Velocity, RPMtoRawSensorUnit(velocity));
    } else {
      stopShoot();
    }
  }

  public void stopShoot() {
    m_shooter_left_falcon.stopMotor();//
    // m_shooter_right_falcon.stopMotor();
  }

  /***************************************************
   * RPM(rotate per Second) - UPS(unit per 100 ms)
   ******************************************************/
  public double RPMtoRawSensorUnit(double velocity) {
    return velocity * 2048 / 600;
  }

  /****************** UPS - RPM *************************************/
  public double RawSensorUnittoRPM(double velocity) {
    return velocity / 2048 * 600;
  }

  /*****************************
   * deg : Angle of Car and Shooter (Unit:degree)
   * <p>
   * speedms£º Speed of Car (Unit: mter per seconds)
   * <p>
   * speedx : car speed in shooter drication
   *************************************************/
  public double get_x_speed(double deg, double speed_mps) {
    double speedx = 0;
    speedx = speed_mps * Math.cos(deg * Math.PI / 180);
    return speedx;
  }

  public double get_y_speed(double deg, double speed_mps) {
    double speedy = 0;
    speedy = -speed_mps * Math.sin(deg * Math.PI / 180);
    return speedy;
  }

  /**********************************************************************************************************
   * return the degree of rotation
   * <p>
   * Input: encoder_spin.getPosition()
   * <p>
   * Example:
   * SmartDashboard.putNumber("SpinDeg",spin_encoder_2_deg(encoder_spin.getPosition()));
   ************************************************/
  public double spin_encoder_2_deg(double encoder) {
    double degree = -2 - (encoder / ShooterConstant.rotation_ratio * 360);
    // -2 is the angle of shooter_elevation Zero_to_limitswitch
    return degree;
  }

  /***************************************************
   * IS NO USED,and i dont know where can get 19.33 and 90 and 6
   ******************************************************/
  public double pitch_encoder_2_deg(double canencnum) {
    double val = 0;
    val = 90 - (-canencnum / ShooterConstant.elevation_ratio * 360 + 6); // val = 90 - (-canencnum/20/19.33*360+6)
    return val;
  }

  /***************************************************
   * i dont know where can get 19.33
   ******************************************************/
  public double degree_2_encoder(double degree) {
    double aval = 0;
    aval = -(((90 - degree) - 6) / 360 * 19.33 * 20);
    return aval;
  }

  /***************************************************
   * Return the distance between car and target Pipelines
   * <p>
   * Camera_getPitch: result.getBestTarget().getPitch();
   * <p>
   * return distance: Meters
   ******************************************************/
  private double get_distance_to_Target(double Camera_getPitch) {
    double dis_error = 0.45; // the error of return and reality
    double distance = PhotonUtils.calculateDistanceToTargetMeters(PhotonVisionConstant.Camera_height,
        PhotonVisionConstant.Target_height,
        PhotonVisionConstant.Camera_pitch,
        Camera_getPitch * Math.PI / 180)
        + dis_error;
    SmartDashboard.putNumber("TargetDistance", distance);
    return distance;
  }

  /***************************************************
   * Return the Angle and Speed with parabola calculate,you can get the process on
   * README.MD
   * <p>
   * distance_to_Target : Meter
   * <p>
   * result[0] = angle : degree
   * <p>
   * result[1] = velocity : Meter per seconds
   ******************************************************/
  private static double[] calculate_parabola(double distance_to_Target) {
    double[] result = new double[2];

    double x1 = distance_to_Target;
    double y1 = PhotonVisionConstant.Camera_height - PhotonVisionConstant.Target_height;
    double x2 = distance_to_Target - 0.2;
    double y2 = y1 + 0.1;

    double parabola_a = ((y1 * x2 / x1) - y2) / ((x1 * x2) - (x2 * x2));
    double parabola_b = (y1 - (x1 * x1 * parabola_a)) / x1;

    double vertex_x = -parabola_b / (2 * parabola_a);
    double vertex_y = (-parabola_b * parabola_b) / (4 * parabola_a);

    double angle = (Math.atan((2 * vertex_y) / vertex_x)) * 180 / Math.PI;

    double x = (vertex_x) * 2;
    double fenzi = x * x * 9.8;
    double fenmu = Math.sin(3.14 / 180 * 2 * angle) * Math.cos(3.14 / 180 * angle);

    double velocity = Math.pow((fenzi / fenmu), (1.0 / 3.0)) * 3 - 0.15;

    result[0] = angle;
    result[1] = velocity;
    return result;
  }

  /***************************************************
   * <p>
   * targetVelocity : UnitsPer100ms
   * <p>
   * return setvelocity : UnitsPer100ms
   ******************************************************/
  private double falcon500_delay(double targetvelocity) {
    double actualvelocity = m_shooter_left_falcon.getSelectedSensorVelocity();
    double setvelocity = 0;
    if ((targetvelocity - actualvelocity) >= 1000)
      setvelocity += 75;
    else if ((targetvelocity - actualvelocity) <= -1000)
      setvelocity -= 75;
    else if (targetvelocity == 0) {
      // if targetvelocity is 0 & speed<600 ,keep the speed
      if (Math.abs(actualvelocity) < 600)
        setvelocity = actualvelocity;
      else
        setvelocity = 0; // ? why the actualvelocity>600 and targetvelocity=0,setvelocity=0
    } else
      setvelocity = targetvelocity;
    SmartDashboard.putNumber("TargetShooterSpeed", setvelocity);
    return setvelocity;
  }

  /***************************************************
   * 
   * 
   ******************************************************/
  public Boolean auto_shoot() {
    double spin_output = 0;
    var result = m_camera.getLatestResult();
    if (result.hasTargets()) {

      spin_output = m_SpinPidController.calculate(result.getBestTarget().getYaw(), 0);

      double distance = get_distance_to_Target(result.getBestTarget().getPitch());
      double target_pitchAngle = calculate_parabola(distance)[0];

      m_elevation_motor.set(m_PitchPidController.calculate(degree_2_encoder(target_pitchAngle), m_elevate_encoder.getPosition()));
      double target_Velocity = calculate_parabola(distance)[1] / ShooterConstant.flywheel_up10ms_mps;
      falcon500_delay(target_Velocity);

      if (Math.abs(degree_2_encoder(target_pitchAngle) - m_elevate_encoder.getPosition()) < 3) {
        // if Pitch is ready
        if (target_Velocity < Math.abs(m_shooter_left_falcon.getSelectedSensorVelocity())) {
          // if Speed is ready
          return true;
        } else
          return false;
      } else
        return false;

    } 
    else {
      spin_output = -0.2;
      return false;
    }
    m_rotate_motor.set(spin_output);
  }

}
