// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import frc.robot.Constants; 

public class Shoulder extends SubsystemBase {
  /** Creates a new Arm. */ 
  private CANSparkMax shoulderMotor;
  private Joystick controller;
  private double shoulderSpeed; 
  private boolean initialized = false; 

  public Shoulder(Joystick c, double s) { 
    //Shoulder Motor Configuration
    //shoulderMotor = new CANSparkMax(Constants.Arm.shoulderID, MotorType.kBrushless);  
    shoulderMotor.restoreFactoryDefaults();  
    shoulderMotor.setSmartCurrentLimit(Constants.Arm.Shoulder_Motor_Limit);
    shoulderMotor.setIdleMode(IdleMode.kBrake); 
    shoulderMotor.enableVoltageCompensation(Constants.Arm.voltageComp); 
    
    controller = c;
    shoulderSpeed = s;
  }

  //Moving Shoulder With Joystick Code
  public void elevateArmWithJoysticks(){ 
    SmartDashboard.putNumber("Joystick getY: ", controller.getY());
    
    if (controller.getY() > 0.3 ) {
        if (shoulderMotor.getEncoder().getPosition() > -80){
             shoulderMotor.set(-shoulderSpeed*controller.getY()*2.0);
        }
        else
        {
          shoulderMotor.set(0);
        }
    }
    else {
        if (controller.getY() < -0.3){
          shoulderMotor.set(-shoulderSpeed*controller.getY()*2.0);
        }
        else{
          shoulderMotor.set(0);
        }
      }
  }  

  public void elevate(double speed){
    shoulderMotor.set(speed); 
  } 

  public void stop(){
    shoulderMotor.stopMotor(); 
  } 

  public RelativeEncoder getShoulderEncoder(){ 
    return shoulderMotor.getEncoder();
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    elevateArmWithJoysticks(); 
    SmartDashboard.putNumber("Shoulder Position", shoulderMotor.getEncoder().getPosition());  
       
    if(initialized == false){ 
      if(shoulderMotor.getEncoder().getPosition() > 2){ 
        initialized = true; 
      } 
      else{ 
        shoulderMotor.set(Constants.Arm.shoulderSpeed);
      }
    }
  } 
}