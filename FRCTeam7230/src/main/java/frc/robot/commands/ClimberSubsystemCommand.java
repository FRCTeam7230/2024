// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberSubsystemCommand extends Command {
  /** Creates a new ClimberSubsystemCommand. */
  ClimberSubsystem s_ClimberSubsystem;
  Joystick m_mechanismsController;
  int rotationalCoefficient;
  public ClimberSubsystemCommand(ClimberSubsystem climber, Joystick control, int direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_ClimberSubsystem =climber;
    m_mechanismsController = control;
    rotationalCoefficient = direction;
    addRequirements(s_ClimberSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current = s_ClimberSubsystem.getClimbEncoder();
    if(rotationalCoefficient==-1){
      while(s_ClimberSubsystem.getClimbEncoder() > current - 10){
        s_ClimberSubsystem.startClimber(-0.5);
      }
    }else if(rotationalCoefficient==1){
      while(s_ClimberSubsystem.getClimbEncoder() < current + 10){
        s_ClimberSubsystem.startClimber(0.5);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_ClimberSubsystem.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}