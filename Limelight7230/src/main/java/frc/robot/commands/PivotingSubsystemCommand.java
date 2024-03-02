// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotingSubsystem;

public class PivotingSubsystemCommand extends Command {
  /** Creates a new ShooterSubsystemCommand. */
  private PivotingSubsystem s_pivotingSubsystem;
  private Joystick m_mechanismsController;

  SlewRateLimiter pivot_limit = new SlewRateLimiter(0.5);//use this if we need to regulate signal for joystick
  double pivot;
  double rotationalCoefficient;
  public PivotingSubsystemCommand(PivotingSubsystem rotate, Joystick control, int direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_pivotingSubsystem = rotate;
    this.m_mechanismsController = control;
    this.rotationalCoefficient = direction;

    addRequirements(s_pivotingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double current;
    // pivot = m_mechanismsController.getY();
    // if(pivot != 0){//joystick no button
    //   s_pivotingSubsystem.RotateShooter(pivot_limit.calculate(pivot));//does this account for both up and down?
    // }else if(rotationalCoefficient==-1){
    //   current = s_pivotingSubsystem.counterValue();
    //   while(s_pivotingSubsystem.counterValue() > current - 10){
    //     s_pivotingSubsystem.RotateShooter(-0.5);
    //   }
    // }else if(rotationalCoefficient==1){
    //   current = s_pivotingSubsystem.counterValue();
    //   while(s_pivotingSubsystem.counterValue() < current + 10){
    //     s_pivotingSubsystem.RotateShooter(0.5);
    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_pivotingSubsystem.StopRotation();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
