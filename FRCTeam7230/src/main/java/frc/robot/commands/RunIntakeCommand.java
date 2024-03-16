// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.ShooterConstants.kRotationalSpeed;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RunIntakeCommand extends Command {
  /** Creates a new IntakeSubsystemCommand. */
  private IntakeSubsystem s_intakeSubsystem;
  private Joystick m_mechanismsController;
  public RunIntakeCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    // m_mechanismsController = controller;
    s_intakeSubsystem = intake;
    addRequirements(s_intakeSubsystem);
    
    // addCommands(
    //   s_intakeSubsystem.startIntake(kRotationalSpeed).withTimeout(5)
    //   .until(() -> IntakeSubsystem.checkSensor()),
    //   // ,Commands.waitUntil(() -> IntakeSubsystem.checkSensor()),
    //   // ,Commands.waitSeconds(4),
    //   s_intakeSubsystem.stopIntake()
    // );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_intakeSubsystem.startIntakeSystem();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(m_mechanismsController.getRawButtonReleased(4)||m_mechanismsController.getRawButtonReleased(4)){
    //   s_intakeSubsystem.stopIntakeSystem();
    // }
    // else{
      
    // }


    // s_intakeSubsystem.startIntakeSystem();
    // System.out.println(!IntakeSubsystem.checkSensor());
    // Commands.sequence(
      // new InstantCommand(() -> System.out.println(!IntakeSubsystem.checkSensor())),
      // new WaitCommand(1)
      // );
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_intakeSubsystem.stopIntakeSystem();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("isFinished is running");
    return IntakeSubsystem.checkSensor();
    // return false;
  }
}
