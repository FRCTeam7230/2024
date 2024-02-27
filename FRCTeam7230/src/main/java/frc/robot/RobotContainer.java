// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//autonomous goes here

package frc.robot;

import frc.robot.commands.ClimberSubsystemCommand;
import frc.robot.commands.IntakeSubsystemCommand;
import frc.robot.commands.PivotingSubsystemCommand;
import frc.robot.subsystems.ClimberSubsystem;
//import frc.robot.subsystems.AutosSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PivotingSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer{
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem s_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem s_shooterSubsystem = new ShooterSubsystem();
  private final PivotingSubsystem s_pivotingSubsystem = new PivotingSubsystem();
  private final Limelight s_Limelight = new Limelight();
  private final Joystick m_mechanismsController = new Joystick(0);
  private final ClimberSubsystem s_ClimberSubsystem = new ClimberSubsystem();
  //private final AutosSubsystem a_autos = new AutosSubsystem();

  private final Joystick driveJoystick = Mechanisms.m_driverController;
  private final Joystick mechJoystick = Mechanisms.m_mechanismsController;
  private JoystickButton intakeButton = new JoystickButton(m_mechanismsController, 1);//is it always going to be autnonmous?
  private JoystickButton PivotUpButton = new JoystickButton(m_mechanismsController, 2);
  private JoystickButton PivotDownButton = new JoystickButton(m_mechanismsController, 3);//is it always going to be autnonmous?
  private JoystickButton ClimberUpButton = new JoystickButton(m_mechanismsController, 4);
  private JoystickButton ClimberDownButton = new JoystickButton(m_mechanismsController, 5);
  
  //is it always going to be autnonmous?
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    defaultCommands(); 
  }

  private void configureBindings() {//for manual input (buttons)
    PivotUpButton.whileTrue(new PivotingSubsystemCommand(s_pivotingSubsystem, m_mechanismsController, 1));
    PivotDownButton.whileTrue(new PivotingSubsystemCommand(s_pivotingSubsystem, m_mechanismsController, -1));
    ClimberUpButton.whileTrue(new ClimberSubsystemCommand(s_ClimberSubsystem, m_mechanismsController, 1));
    ClimberDownButton.whileTrue(new ClimberSubsystemCommand(s_ClimberSubsystem, m_mechanismsController, -1));
    intakeButton.whileTrue(new IntakeSubsystemCommand(s_intakeSubsystem));
    
  }
  public void defaultCommands(){ //for anything that runs constantly
    s_pivotingSubsystem.setDefaultCommand(new PivotingSubsystemCommand(s_pivotingSubsystem, m_mechanismsController,1));
    //s_Limelight.setDefaultCommand(new Limelight());
    

   


  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void teleopInit(){
  
    //add wait function for about 4 seconds
      //go into original subsystem file and check encoder value encoder -= 10 return the encoder value 
  }
  public void autnomousInit() {
    // An example command will be run in autonomous
    // return a_autos.soontobemadecommand();
  }
}
