// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechanismsControllerPort = 0;
  }

  public static final class DriveConstants {
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kRearRightDrivingCanId = 1;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearRightTurningCanId = 2;
  }

  public static final class ShooterConstants {
    public static final int kShooterMotorId = 1;
    public static final int kShooterPivotMotorId = 1;
    public static final int kNoteBeamSensorId = 1;
    public static final int kPivotEncoderId = 1;
    public static final int kUpperLimitSwitchId = 1;

    public static final double kRotationalSpeed = 0.5;
    public static final double kDegreesPerStep = 1;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorId = 1;
    public static final int kTransferToShooterMotorId = 1;
  }

  public static final class ClimbingConstants {
    public static final int kClimbingMotorId = 1;
    public static final int kClimbingEncoderId = 1;
  }
  
}
