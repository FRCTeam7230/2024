// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystemCommand extends Command {
  /** Creates a new IntakeSubsystemCommand. */
  private IntakeSubsystem s_intakeSubsystem;
  public IntakeSubsystemCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_intakeSubsystem = intake;
    addRequirements(s_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_intakeSubsystem.startIntakeSystem();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_intakeSubsystem.stopIntakeSystem();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_intakeSubsystem.checkSensor();
  }
}
