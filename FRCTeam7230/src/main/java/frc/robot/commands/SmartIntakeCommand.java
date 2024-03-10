// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.Constants.AutoConstants;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// /** An example command that uses an example subsystem. */
// public class SmartIntakeCommand extends SequentialCommandGroup {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final DriveSubsystem m_driveSubsystem;
//   private final VisionSubsystem m_visionSubsystem;
//   private final IntakeSubsystem m_IntakeSubsystem;
  

//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public SmartIntakeCommand(DriveSubsystem subsystem, VisionSubsystem subsystem2, IntakeSubsystem subsystem3) {
//     m_driveSubsystem = subsystem;
//     m_visionSubsystem = subsystem2;
//     m_IntakeSubsystem = subsystem3;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(subsystem);
//     addRequirements(subsystem2);
//   addCommands(
//     new SmartRotateCommand(m_driveSubsystem,m_visionSubsystem),
//     new RunIntakeCommand(m_IntakeSubsystem));
//   }
// }
