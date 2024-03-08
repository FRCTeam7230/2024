// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystemSim;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PivotingSubsystem;
//import frc.robot.commands.IntakeSubsystemCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunShooterCommand;
//import frc.robot.commands.PivotingSubsystemCommand;
//import frc.robot.commands.ShooterSubsystemCommand;
//import frc.robot.commands.SmartIntakeCommand;

public class Autos {
    private final SendableChooser<Command> autoChooser;
    private ShooterSubsystem shootersubsystem;
    private IntakeSubsystem intakesubsystem;

    public Autos(SwerveSubsystemSim subsystem) {
        NamedCommands.registerCommand("marker1", Commands.run(() -> new RunShooterCommand(shootersubsystem)));
        NamedCommands.registerCommand("marker2", Commands.run(() ->new RunIntakeCommand(intakesubsystem)));
        //after running the RunShooter and RunIntake, test out SmartShooter and SmartIntake
        //*** 
        //NamedCommands.registerCommand("print hello", Commands.print("hello"));

        // Configure AutoBuilder
        AutoBuilder.configureHolonomic(
                subsystem::getPose,
                subsystem::resetPose,
                subsystem::getSpeeds,
                subsystem::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0, 0), // Translation constants
                        new PIDConstants(5.0, 0, 0), // Rotation constants
                        subsystem.getMaxModuleSpeed(),
                        subsystem.getNorm(), // Drive base radius (distance from center to furthest module)
                        new ReplanningConfig()),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        //System.out.println("Alliance is present: " + alliance.get());
                        return alliance.get() == DriverStation.Alliance.Red;
                    } else {
                        //System.out.println("No alliance.");
                        return false;
                    }
                },
                subsystem);

        // Add a button to run the example auto to SmartDashboard, this will also be in
        // the auto chooser built above
        SmartDashboard.putData("Autonomous Path Middle", new PathPlannerAuto("Example Auto"));
        SmartDashboard.putData("Autonomous Path Left", new PathPlannerAuto("New New Auto"));
        SmartDashboard.putData("Autonomous Path Right", new PathPlannerAuto("New Auto"));


        // Add a button to run pathfinding commands to SmartDashboard
        SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
                new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
                new PathConstraints(
                        4.0, 4.0,
                        Units.degreesToRadians(360), Units.degreesToRadians(540)),
                0,
                2.0));
        SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
                new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
                new PathConstraints(
                        4.0, 4.0,
                        Units.degreesToRadians(360), Units.degreesToRadians(540)),
                0,
                0));

        // Add a button to SmartDashboard that will create and follow an on-the-fly path
        // This example will simply move the robot 2m in the +X field direction
        SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
            Pose2d currentPose = subsystem.getPose();

            // The rotation component in these poses represents the direction of travel
            Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
            Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)),
                    new Rotation2d());

            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
            PathPlannerPath path = new PathPlannerPath(
                    bezierPoints,
                    new PathConstraints(
                            4.0, 4.0,
                            Units.degreesToRadians(360), Units.degreesToRadians(540)),
                    new GoalEndState(0.0, currentPose.getRotation()));

            // Prevent this path from being flipped on the red alliance, since the given
            // positions are already correct
            path.preventFlipping = true;

            AutoBuilder.followPath(path).schedule();
        }));

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            subsystem.m_field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            subsystem.m_field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            subsystem.m_field.getObject("path").setPoses(poses);
        });

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
