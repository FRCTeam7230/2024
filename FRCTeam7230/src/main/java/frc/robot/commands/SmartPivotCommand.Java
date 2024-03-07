// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.PivotingSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class SmartPivotCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final PivoitngSubsystem m_pivotSubsystem;
  private boolean finishedPivoting = false;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SmartPivotCommand(PivotingSubsystem subsystem, VisionSubsystem subsystem2) {
    m_pivotingSubsystem = subsystem;
    m_visionSubsystem = subsystem2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(subsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finishedPivoting = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] visionData = m_visionSubsystem.captureTask();
    if (visionData[0] > kSmartPivotingAngleMargin){
        m_pivotingSubsystem.rotateShooter(kSmartPivotSpeed);
      }
    else if (visionData[0] < -kSmartPivotingAngleMargin){
        m_pivotingSubsystem.rotateShooter(kSmartPivotSpeed);
      }
    else {
      m_pivotingSubsystem.stopRotation();
      finishedPivoting = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedPivoting;
  }
}
