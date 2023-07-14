
package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  /******************* SUBSYSTEM INIT **************************/
  final DriveSubsystem m_driver = new DriveSubsystem();//
  private final ShooterSubsystem m_shooter = new ShooterSubsystem(); //
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final DeliverSubsystem m_deliver = new DeliverSubsystem();
  private final DashaboardSubsystem m_Dashaboard = new DashaboardSubsystem();

  private final Command m_GoStrightCommand = new DoStrightCommand(1.2, 1.2, m_driver);
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  /*********************CONTRALLER BUTTON INIT************************** */
  private Joystick m_Joystick = new Joystick(Contorller.joystickID);
  private XboxController m_Xbox = new XboxController(Contorller.xboxID);

  private final JoystickButton UpButtonY = new JoystickButton(m_Xbox, 4);
  private final JoystickButton DownButtonA = new JoystickButton(m_Xbox, 1);
  private final JoystickButton launchButton = new JoystickButton(m_Joystick, 1);

  private POVButton m_rotate_colock_button = new POVButton(m_Xbox, 90);
  private POVButton m_rotate_uncolock_button = new POVButton(m_Xbox, 270);
  private POVButton m_elevation_clock_button = new POVButton(m_Xbox, 0);
  private POVButton m_elevation_unclock_button = new POVButton(m_Xbox, 180);

  /*********************SHUFFLEBOARD************************** */
  ShuffleboardTab ShooterTab = Shuffleboard.getTab("ShooterTab");     //selectTab("ShooterTab")
  NetworkTableEntry shooterEnable = ShooterTab.add("Shooter Enable", false).getEntry();
  
  /*********************AUTOMOUSE CHOOSER************************** */
  // SendableChooser<Command> m_chooser = new SendableChooser<>();

  /***************************************************************************************/
  public RobotContainer() {
    configureButtonBindings();

    m_driver.setDefaultCommand(new ArcadeDriverCommand(m_driver,
        () -> m_Xbox.getLeftY(),
        () -> m_Xbox.getRightX()));
    m_intake.setDefaultCommand(new IntakeCommand(m_intake, m_deliver,
        () -> m_Xbox.getLeftTriggerAxis(),
        () -> m_Xbox.getRightTriggerAxis()));
    m_shooter.setDefaultCommand(new ShooterCommand(m_shooter,
        () -> m_Joystick.getRawAxis(3),
        () -> m_Joystick.getRawButtonPressed(Contorller.JoystickVersionButtin)));

    SmartDashboard.putNumber("Xbox_Axis_L2", m_Xbox.getRightTriggerAxis());
    SmartDashboard.putNumber("Joystick_Axis_Trigle", m_Joystick.getRawAxis(3));

    m_chooser.setDefaultOption("Simple Auto", m_GoStrightCommand);
    // m_chooser.addOption("Complex Auto", m_complexAuto);
    Shuffleboard.getTab("Autonomous").add(m_chooser);
     // Log Shuffleboard events for command initialize, execute, finish, interrupt
     CommandScheduler.getInstance()
     .onCommandInitialize(
         command ->
             Shuffleboard.addEventMarker(
                 "Command initialized", command.getName(), EventImportance.kNormal));
 CommandScheduler.getInstance()
     .onCommandExecute(
         command ->
             Shuffleboard.addEventMarker(
                 "Command executed", command.getName(), EventImportance.kNormal));
 CommandScheduler.getInstance()
     .onCommandFinish(
         command ->
             Shuffleboard.addEventMarker(
                 "Command finished", command.getName(), EventImportance.kNormal));
 CommandScheduler.getInstance()
     .onCommandInterrupt(
         command ->
             Shuffleboard.addEventMarker(
                 "Command interrupted", command.getName(), EventImportance.kNormal));
  }

  private void configureButtonBindings() {
    // new NetworkButton(shooterEnable).whenPressed (new InstantCommand(m_shooter::enable));

    UpButtonY   .whenPressed(new InstantCommand(m_intake::InakeUp, m_intake));
    DownButtonA .whenPressed(new InstantCommand(m_intake::intakeDown, m_intake));
    launchButton.whenPressed(new InstantCommand(m_shooter::launch, m_shooter) )
                .whenReleased(new InstantCommand(m_shooter::stopLaunch));
    // new JoystickButton(m_Joystick,1).whenPressed(new InstantCommand(m_shooter::launch, m_shooter) )
    //                                 .whenReleased(new InstantCommand(m_shooter::stopLaunch));

    m_rotate_colock_button    .whileHeld(new InstantCommand(m_shooter::rotate_clock, m_shooter))
                              .whenReleased(new InstantCommand(m_shooter::rotate_stop, m_shooter));
    m_rotate_uncolock_button  .whileHeld(new InstantCommand(m_shooter::rotate_unclock, m_shooter))
                              .whenReleased(new InstantCommand(m_shooter::rotate_stop, m_shooter));
    m_elevation_clock_button  .whileHeld(new InstantCommand(m_shooter::elevation_clock, m_shooter))
                              .whenReleased(new InstantCommand(m_shooter::elevation_stop, m_shooter));
    m_elevation_unclock_button.whileHeld(new InstantCommand(m_shooter::elevation_unclock, m_shooter))
                              .whenReleased(new InstantCommand(m_shooter::elevation_stop, m_shooter));

    
  }

  // ***************************************************************** */
  public void teleopInit() {

  }

  public void teleopPeriodic() {

  }

  public void disabledInit() {
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
