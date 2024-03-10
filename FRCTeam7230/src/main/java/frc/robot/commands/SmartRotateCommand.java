// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
// import static frc.robot.Constants.DriveConstants.*;
// import static frc.robot.Constants.VisionConstants.*;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// /** An example command that uses an example subsystem. */
// public class SmartRotateCommand extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final DriveSubsystem m_driveSubsystem;
//   private final VisionSubsystem m_visionSubsystem;
//   private boolean finishedRotating = false;


//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public SmartRotateCommand(DriveSubsystem subsystem, VisionSubsystem subsystem2) {
//     m_driveSubsystem = subsystem;
//     m_visionSubsystem = subsystem2;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(subsystem);
//     addRequirements(subsystem2);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     finishedRotating = false;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double[] visionData = m_visionSubsystem.captureTask();
//     if (visionData[1] > kSmartDrivingAngleMargin){
//       m_driveSubsystem.rotateUntil(-kSmartRotateSpeed);
//     }
//     else if (visionData[1] < -kSmartDrivingAngleMargin){
//       m_driveSubsystem.rotateUntil(kSmartRotateSpeed);
//     }
//     else {
//       m_driveSubsystem.rotateUntil(0);
//       finishedRotating = true;
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return finishedRotating;
//   }
// }
