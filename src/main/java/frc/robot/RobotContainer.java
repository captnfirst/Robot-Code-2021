// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.ArmIntake.ArmIntakeCommand;
import frc.robot.commands.ArmIntake.IntakeGroupCommand;
//import frc.robot.commands.Auto.Auto6Balls;
import frc.robot.commands.Auto.Autonomous;
import frc.robot.commands.Auto.Default.AutoManuallyCommand;
import frc.robot.commands.Climb.ClimbCommand;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Push.PushCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Trigger.TriggerCommand;
import frc.robot.commands.Turret.TurretControllCommand;
import frc.robot.commands.Turret.TurretGroupCommand;
import frc.robot.subsystems.ArmIntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PushSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpiutil.net.PortForwarder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public Joystick m_driverController = new Joystick(JoystickConstants.F310);
  public Joystick m_operatorController = new Joystick(JoystickConstants.Panel);

  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ArmIntakeSubsystem m_intake = new ArmIntakeSubsystem();
  public final PushSubsystem m_push = new PushSubsystem();
  public final TurretSubsystem m_turret = new TurretSubsystem();
  public final TriggerSubsystem m_trigger = new TriggerSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final ClimbSubsystem m_climb = new ClimbSubsystem();
  public final VisionSubsystem m_vision = new VisionSubsystem();
  public final LedSubsystem m_led = new LedSubsystem();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    PortForwarder.add(5800, "photonvision.local", 5800);
    
    configureButtonBindings();

    //m_robotDrive.setDefaultCommand(new DriveCommand(m_robotDrive,() -> -m_driverController.getRawAxis(1)* 0.8,() -> -m_driverController.getRawAxis(4)* -0.8));
    m_robotDrive.setDefaultCommand(new DriveCommand(m_robotDrive,() -> -m_driverController.getRawAxis(1),() -> m_driverController.getRawAxis(4)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // Top Alma, Taşıma
    new JoystickButton(m_driverController, 6).whileHeld(new IntakeGroupCommand(m_intake, m_push));

    // Top Geri Taşıma
    new JoystickButton(m_driverController, 5).whileHeld(new PushCommand(m_push, 0.8));

    // Top Alma PANEL
    new JoystickButton(m_operatorController, 1).whileHeld(new ArmIntakeCommand(m_intake, 0.4));
    new JoystickButton(m_operatorController, 8).whileHeld(new ArmIntakeCommand(m_intake, -0.4));

    // Top Alma F310
    //new JoystickButton(m_driverController, 1).whileHeld(new ArmIntakeCommand(m_intake, 0.8));
    //new JoystickButton(m_driverController, 3).whileHeld(new ArmIntakeCommand(m_intake, -0.8));

    // Top Taşıma
    new JoystickButton(m_operatorController, 10).whileHeld(new PushCommand(m_push, 0.4));
    new JoystickButton(m_operatorController, 9).whileHeld(new PushCommand(m_push, -0.4));

    // Kafaya Tetik
    new JoystickButton(m_operatorController, 7).whileHeld(new TriggerCommand(m_trigger, -0.4));
    new JoystickButton(m_operatorController, 5).whileHeld(new TriggerCommand(m_trigger, 0.4));

    // Kafa Dönme Auto
    

    new JoystickButton(m_driverController, 1).whileHeld(new TurretControllCommand(m_turret, m_vision));

    // Kafa Dönme Manuel 
    new POVButton(m_driverController, 270).whileHeld(new TurretGroupCommand(m_turret,0.25));
    new POVButton(m_driverController, 90).whileHeld(new TurretGroupCommand(m_turret,-0.25));

    // Top Fırlatma Auto
    new JoystickButton(m_operatorController, 11).whileHeld(new AutoManuallyCommand(m_shooter, m_trigger, m_push, m_turret, m_vision));

    // Top Fırlatma Manuel
    new JoystickButton(m_operatorController, 12).whileHeld(new ShooterCommand(m_shooter,-1));

    // Encoder
    //new JoystickButton(m_driverController, 4).whenPressed(new EncoderValueCommand(m_head));

    // Tırmanma
    new JoystickButton(m_driverController, 8).whileHeld(new ClimbCommand(m_climb, 1));
    new JoystickButton(m_driverController, 7).whileHeld(new ClimbCommand(m_climb, -1));

    // Gyro
    //new JoystickButton(m_driverController, 1).whileHeld(new gyrocommand(m_gyro));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new AutoCommand(m_robotDrive, m_shooter, m_trigger, m_push);
    //return new Auto6Balls(m_robotDrive,m_vision, m_intake, m_turret, m_shooter, m_trigger, m_push, m_led);
    return new Autonomous(m_turret, m_vision, m_shooter, m_trigger, m_push, m_robotDrive, m_intake);
  }
}
