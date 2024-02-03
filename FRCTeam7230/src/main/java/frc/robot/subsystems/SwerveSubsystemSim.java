// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveSubsystemSim extends SwerveSubSystemBase {
    public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
    public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
    public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
    public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);
    public static final double maxModuleSpeed = 4.5; // M/S

    private SimSwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;

    private SimGyro gyro;

    public SwerveSubsystemSim() {
        gyro = new SimGyro();

        modules = new SimSwerveModule[] {
                new SimSwerveModule(),
                new SimSwerveModule(),
                new SimSwerveModule(),
                new SimSwerveModule()
        };

        kinematics = new SwerveDriveKinematics(
                flModuleOffset,
                frModuleOffset,
                blModuleOffset,
                brModuleOffset);

        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {
        // Update the simulated gyro, not needed in a real project
        gyro.updateRotation(getSpeeds().omegaRadiansPerSecond);

        odometry.update(gyro.getRotation2d(), getPositions());

        m_field.setRobotPose(getPose());
    }

    public void printPose() {
        Pose2d m_pose = getPose();
        System.out.println("x: " + m_pose.getX() + ", y: " + m_pose.getY());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        // System.out.print("Before: ");
        // printPose();

        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);

        // printModuleStates(targetStates);

        setStates(targetStates);

        // printModulePositions();

        // System.out.print("After: ");
        // printPose();
    }

    public void printModuleStates(SwerveModuleState[] states) {
        System.out.println("States: ");
        System.out.println(states[0]);
        System.out.println(states[1]);
        System.out.println(states[2]);
        System.out.println(states[3]);
    }

    // Prints the angular position of the swerve drive modules
    public void printModulePositions() {
        for (int i = 0; i < modules.length; i++) {
            System.out.println(modules[i].getPosition());
        }
    }

    public void setStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, maxModuleSpeed);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(targetStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public double getNorm() {
        return flModuleOffset.getNorm();
    }

    public double getMaxModuleSpeed() {
        return maxModuleSpeed;
    }

    /**
     * Basic simulation of a swerve module, will just hold its current state and not
     * use any hardware
     */
    class SimSwerveModule {
        private SwerveModulePosition currentPosition = new SwerveModulePosition();
        private SwerveModuleState currentState = new SwerveModuleState();

        public SwerveModulePosition getPosition() {
            return currentPosition;
        }

        public SwerveModuleState getState() {
            return currentState;
        }

        public void setTargetState(SwerveModuleState targetState) {
            // Optimize the state
            currentState = SwerveModuleState.optimize(targetState, currentState.angle);

            currentPosition = new SwerveModulePosition(
                    currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
        }
    }

    /**
     * Basic simulation of a gyro, will just hold its current state and not use any
     * hardware
     */
    class SimGyro {
        private Rotation2d currentRotation = new Rotation2d();

        public Rotation2d getRotation2d() {
            return currentRotation;
        }

        public void updateRotation(double angularVelRps) {
            currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02));
        }
    }
}
