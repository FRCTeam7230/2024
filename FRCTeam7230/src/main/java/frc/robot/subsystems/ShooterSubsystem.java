// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.SparkRelativeEncoder.Type;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Mechanisms;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.NeoMotorConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ShooterSubsystem extends SubsystemBase{

  
  /** Variables */
  private static DigitalInput intakeSensor = Mechanisms.m_noteBeamSensor;
  // private static DigitalInput limitSwitch = Mechanisms.m_upperLimitSwitch; //povit low
  private CANSparkMax shooterIntakeMotor = Mechanisms.m_ShooterIntakeMotor;

  private static CANSparkMax rightShooterMotor = Mechanisms.m_rightShooterMotor;
  private static CANSparkMax leftShooterMotor = Mechanisms.m_leftShooterMotor;


  public static boolean shooterOn = false;

  public ShooterSubsystem() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
 

  public void initShooting(double rotSpeed){
    rightShooterMotor.set(-rotSpeed);//check experimentally what the velocity is at a motorRotateSpeed Voltage
    leftShooterMotor.set(rotSpeed);
    shooterIntakeMotor.set(rotSpeed);
    Commands.waitSeconds(5);
    System.out.println("waited");
    shooterIntakeMotor.set(0);
    shooterIntakeMotor.stopMotor();
    rightShooterMotor.set(-rotSpeed);
    leftShooterMotor.set(rotSpeed);
    Commands.waitSeconds(5);
    shooterIntakeMotor.set(-rotSpeed);
  }
  

  public void StartShooter(double motorRotateSpeed) {
    


    
    rightShooterMotor.set(-motorRotateSpeed);//check experimentally what the velocity is at a motorRotateSpeed Voltage
    leftShooterMotor.set(motorRotateSpeed);

  }
  
  public boolean checkShooterAtMaxSpeed(){
        if(rightShooterMotor.getEncoder().getVelocity() == (kMotorVoltsToRPM) 
    && leftShooterMotor.getEncoder().getVelocity()== (kMotorVoltsToRPM)){
     return true;
    }
    else{
      return false;
    }
  }

  public void printMotorEncoder(){
    System.out.println(rightShooterMotor.getEncoder().getVelocity());
    System.out.println(leftShooterMotor.getEncoder().getVelocity());
  }

  public void StartShooterIntake(double rotSpeed){
    shooterIntakeMotor.set(-rotSpeed);
  }

  public void StopShooter() {
    rightShooterMotor.stopMotor();
    leftShooterMotor.stopMotor();
  }

  public void StopShooterIntake(){
    shooterIntakeMotor.stopMotor();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
