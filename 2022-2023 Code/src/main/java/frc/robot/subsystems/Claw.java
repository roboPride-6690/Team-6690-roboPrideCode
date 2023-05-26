// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase; 
//import com.ctre.phoenix.motorcontrol.ControlMode;

//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.Compressor; 
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.CompressorConfigType;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  //public static AnalogInput pressureSensor = null; 

  public static DoubleSolenoid claw;  
  private static boolean clawOpen; 
  private static int counter = 0; 
  private static boolean initialized = false;

  /** Creates a new Claw. */
  public Claw(int port, PneumaticsModuleType moduleType) {
      //pressureSensor = new AnalogInput(Constants.Arm.SENSOR_AIR_PRESSURE); 
      claw = new DoubleSolenoid(16, PneumaticsModuleType.REVPH, Constants.Arm.forwardChannel, Constants.Arm.reverseChannel);
      
      CloseClaw(); 

    }

  //public double getPressure() {
    //return 250 * (pressureSensor.getVoltage() / 4.82) - 25;
  //} 

  public void OpenClaw() {
    claw.set(DoubleSolenoid.Value.kReverse);       
    clawOpen = true; 
  } 

  public void CloseClaw() {
    claw.set(DoubleSolenoid.Value.kForward); 
    clawOpen = false;  
  } 

  public boolean clawState(){ 
    return clawOpen;
  }

  public void enableClaw(){ 
    //claw.set(null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run   
    if(initialized == false){
      counter += 1; 
      if(counter > 500){ 
        CloseClaw(); 
        initialized = true;
      }
    }
  } 

  public void stop(){ 
    //claw.
  }
}