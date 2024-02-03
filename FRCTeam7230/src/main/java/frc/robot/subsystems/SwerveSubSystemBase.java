// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubSystemBase extends SubsystemBase {
    public Field2d m_field = new Field2d();

    /** Creates a new ExampleSubsystem. */
    public SwerveSubSystemBase() {
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    }

    public void setX() {
    }

    public void printModulePositions() {
    }

    public void setZero() {
    }

    public void resetOdometry(Pose2d pose) {
    }

    public Pose2d getPose() {
        return null;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
    }

    public double getNorm() {
        return 0.0;
    }

    public ChassisSpeeds getSpeeds() {
        return null;
    }

    public void resetPose(Pose2d pose) {
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    }

    public double getMaxModuleSpeed() {
        return 0.0;
    }
}
