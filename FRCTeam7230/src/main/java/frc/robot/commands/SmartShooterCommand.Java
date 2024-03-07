// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.PivotingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class SmartShooterCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PivotingSubsystem m_pivotingSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SmartShooterCommand(PivotingSubsystem subsystem, VisionSubsystem subsystem2, ShooterSubsystem subsystem3, DriveSubsystem subsystem4) {
    m_pivotingSubsystem = subsystem;
    m_visionSubsystem = subsystem2;
    m_shooterSubsystem = subsystem3;
    m_driveSubsystem = subsystem4;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(subsystem2);
  addCommands(
    new SmartRotateCommand(m_driveSubsystem,m_visionSubsystem),
    new SmartPivotCommand(m_pivotingSubsystem,m_visionSubsystem),
    new RunShooterCommand(m_shooterSubsystem));
  }
}
