// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.ShooterConstants.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BackupShooterCommand extends Command {
  /** Creates a new RunShooterCommand. */
  private ShooterSubsystem s_shooterSubsystem;
  
  private Timer timer = new Timer();

  public BackupShooterCommand(ShooterSubsystem shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_shooterSubsystem = shoot;
    addRequirements(s_shooterSubsystem);

    // addCommands(
    //   s_shooterSubsystem.startShooter(kRotationalSpeed).withTimeout(5)
    //   .until(() -> ShooterSubsystem.checkShooter()),
    //   // ,Commands.waitUntil(() -> ShooterSubsystem.checkShooter()),
    //   // ,Commands.waitSeconds(4),
    //   s_shooterSubsystem.startShooterIntake(kRotationalSpeed).withTimeout(3),
    //   Commands.waitSeconds(2),
    //   s_shooterSubsystem.stopShooter()
    // );


  }

  /*
   * Run Shooting Sequence:
   * 1. Run shooter at max speed outward
   * 2. Run shooterintake out when at max shooter speed
   */

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {
    // if (ShooterSubsystem.checkSensor()){
    //   s_shooterSubsystem.StartShooter(kRotationalSpeed);
    //   timer.reset();
    //   timer.start();
    // }
    
    
   }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(ShooterSubsystem.checkShooter()){
    //   s_shooterSubsystem.StartShooterIntake(kRotationalSpeed);
    // }
    // while(!ShooterSubsystem.checkSensor()){
      s_shooterSubsystem.StartShooterIntake(-kRotationalSpeed);
    // }
  }
    
    // s_shooterSubsystem.printMotorEncoder();
    // new WaitCommand(10.0);
    // s_shooterSubsystem.StopShooter();
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_shooterSubsystem.StopShooter();
    s_shooterSubsystem.StopShooterIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return ShooterSubsystem.checkSensor();
    return false;
  }
}
