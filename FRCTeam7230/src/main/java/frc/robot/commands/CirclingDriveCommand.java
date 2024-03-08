// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.VisionConstants.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Joystick;

/** An example command that uses an example subsystem. */
public class CirclingDriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  private final boolean circlingMode;
  private final Joystick m_driverStick;



  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CirclingDriveCommand(DriveSubsystem subsystem, VisionSubsystem subsystem2, Joystick joystick, boolean mode) {
    m_driveSubsystem = subsystem;
    m_visionSubsystem = subsystem2;
    circlingMode = mode;
    m_driverStick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(subsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] visionData = m_visionSubsystem.captureTask();
    m_driveSubsystem.circlingDrive(circlingMode, m_driverStick.getX(), visionData[0], visionData[1]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
