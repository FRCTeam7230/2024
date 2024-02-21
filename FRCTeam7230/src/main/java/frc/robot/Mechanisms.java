package frc.robot;

import frc.robot.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Mechanisms {
    //General
    public static final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

    //Swerve Drive Subsystem
    public static final CANSparkMax rearLeftTurningMotor = new CANSparkMax(DriveConstants.kRearLeftTurningCanId, MotorType.kBrushless);
    public static final CANSparkMax rearLeftDrivingMotor = new CANSparkMax(DriveConstants.kRearLeftDrivingCanId, MotorType.kBrushless);

    public static final CANSparkMax rearRightTurningMotor = new CANSparkMax(DriveConstants.kRearRightTurningCanId, MotorType.kBrushless);
    public static final CANSparkMax rearTightDrivingMotor = new CANSparkMax(DriveConstants.kRearRightDrivingCanId, MotorType.kBrushless);

    public static final CANSparkMax frontLeftTurningMotor = new CANSparkMax(DriveConstants.kFrontLeftTurningCanId, MotorType.kBrushless);
    public static final CANSparkMax frontLeftDrivingMotor = new CANSparkMax(DriveConstants.kFrontLeftDrivingCanId, MotorType.kBrushless);

    public static final CANSparkMax frontRightTurningMotor = new CANSparkMax(DriveConstants.kFrontRightTurningCanId, MotorType.kBrushless);
    public static final CANSparkMax frontRightDrivingMotor = new CANSparkMax(DriveConstants.kFrontRightDrivingCanId, MotorType.kBrushless);

    //Shooter Subsystem
    public static final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorId, MotorType.kBrushless);
    public static final DigitalInput noteBeamSensor = new DigitalInput(ShooterConstants.kNoteBeamSensorId);

    public static final CANSparkMax shooterPivotMotor = new CANSparkMax(ShooterConstants.kShooterPivotMotorId, MotorType.kBrushless);
    public static final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(ShooterConstants.kPivotEncoderId);
    public static final DigitalInput upperLimitSwitch = new DigitalInput(ShooterConstants.kUpperLimitSwitchId);    

    //Intake Subsystem
    public static final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
    public static final CANSparkMax transferToShooterMotor = new CANSparkMax(IntakeConstants.kTransferToShooterMotorId, MotorType.kBrushless);

    //Climbing Subsystem
    public static final CANSparkMax climbingMotor = new CANSparkMax(ClimbingConstants.kClimbingMotorId, MotorType.kBrushless);
    public static final DutyCycleEncoder climberEncoder = new DutyCycleEncoder(ClimbingConstants.kClimbingEncoderId);
}
