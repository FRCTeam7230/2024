package frc.robot;

import frc.robot.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Mechanisms {
    /*
    //General
    public static final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
    public static final Joystick m_mechanismsController = new Joystick(OperatorConstants.kMechanismsControllerPort);

    //Swerve Drive Subsystem
    public static final CANSparkMax m_rearLeftTurningMotor = new CANSparkMax(DriveConstants.kRearLeftTurningCanId, MotorType.kBrushless);
    public static final CANSparkMax m_rearLeftDrivingMotor = new CANSparkMax(DriveConstants.kRearLeftDrivingCanId, MotorType.kBrushless);

    public static final CANSparkMax m_rearRightTurningMotor = new CANSparkMax(DriveConstants.kRearRightTurningCanId, MotorType.kBrushless);
    public static final CANSparkMax m_rearTightDrivingMotor = new CANSparkMax(DriveConstants.kRearRightDrivingCanId, MotorType.kBrushless);

    public static final CANSparkMax m_frontLeftTurningMotor = new CANSparkMax(DriveConstants.kFrontLeftTurningCanId, MotorType.kBrushless);
    public static final CANSparkMax m_frontLeftDrivingMotor = new CANSparkMax(DriveConstants.kFrontLeftDrivingCanId, MotorType.kBrushless);

    public static final CANSparkMax m_frontRightTurningMotor = new CANSparkMax(DriveConstants.kFrontRightTurningCanId, MotorType.kBrushless);
    public static final CANSparkMax m_frontRightDrivingMotor = new CANSparkMax(DriveConstants.kFrontRightDrivingCanId, MotorType.kBrushless);

    //Shooter Subsystem
    public static final CANSparkMax m_rightShooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorId, MotorType.kBrushless);
    public static final CANSparkMax m_leftShooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorId, MotorType.kBrushless);
    public static final DigitalInput m_noteBeamSensor = new DigitalInput(ShooterConstants.kNoteBeamSensorId);

    public static final CANSparkMax m_shooterPivotMotor = new CANSparkMax(ShooterConstants.kShooterPivotMotorId, MotorType.kBrushless);
    public static final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(ShooterConstants.kPivotEncoderId);
    public static final DigitalInput m_upperLimitSwitch = new DigitalInput(ShooterConstants.kUpperLimitSwitchId);    

    //Intake Subsystem
    public static final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
    public static final CANSparkMax m_transferToShooterMotor = new CANSparkMax(IntakeConstants.kTransferToShooterMotorId, MotorType.kBrushless);

    //Climbing Subsystem
    public static final CANSparkMax m_climbingMotor = new CANSparkMax(ClimbingConstants.kClimbingMotorId, MotorType.kBrushless);
    public static final DutyCycleEncoder m_climberEncoder = new DutyCycleEncoder(ClimbingConstants.kClimbingEncoderId);
    */
}
