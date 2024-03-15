// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.ShooterConstants.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RunShooterCommand extends SequentialCommandGroup {
  /** Creates a new RunShooterCommand. */
  private ShooterSubsystem s_shooterSubsystem;

  public RunShooterCommand(ShooterSubsystem shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_shooterSubsystem = shoot;
    addRequirements(s_shooterSubsystem);

    addCommands(
      s_shooterSubsystem.startShooter(kRotationalSpeed)
      .until(() -> ShooterSubsystem.checkShooter()),
      // Commands.waitUntil(() -> ShooterSubsystem.checkShooter()),
      // Commands.waitSeconds(4),
      s_shooterSubsystem.startShooterIntake(kRotationalSpeed).withTimeout(3)
    );

  }

  /*
   * Run Shooting Sequence:
   * 1. Run shooter at max speed outward
   * 2. Run shooterintake out when at max shooter speed
   */

}
