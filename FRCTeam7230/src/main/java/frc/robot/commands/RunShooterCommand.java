// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.ShooterConstants.*;

public class RunShooterCommand extends Command {
  /** Creates a new RunShooterCommand. */
  private ShooterSubsystem s_shooterSubsystem;
  
  public RunShooterCommand(ShooterSubsystem shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_shooterSubsystem = shoot;
    addRequirements(s_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_shooterSubsystem.checkShooterAtMaxSpeed()){
      s_shooterSubsystem.StartShooterIntake(0.5);
    }
    s_shooterSubsystem.printMotorEncoder();
    // new WaitCommand(10.0);
    // s_shooterSubsystem.StopShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_shooterSubsystem.StopShooter();
    s_shooterSubsystem.StopShooterIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
