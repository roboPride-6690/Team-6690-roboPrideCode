// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;  
//import edu.wpi.first.wpilibj.CompressorConfigType;   
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants; 
//import edu.wpi.first.hal.REVPHJNI;
//import edu.wpi.first.wpilibj.PneumaticHub; 
//import edu.wpi.first.wpilibj.PowerDistribution;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;  
//import edu.wpi.first.wpilibj.PneumaticsControlModule;
//import PHC


public class CompressorSub extends SubsystemBase { 
  Compressor compress; 
  int executeCounter = 0; 
  int pauseCounter = 200;  
  boolean m_Stop = false;
  /** Creates a new Compressor. */
  public CompressorSub() {
    compress = new Compressor(16, PneumaticsModuleType.REVPH); 
    m_Stop = false;
    compress.disable();
    compress.getPressureSwitchValue();

    //compress.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    if(compress.getPressureSwitchValue()){
      if(executeCounter < 300){ 
        compress.enableDigital(); 
        executeCounter += 1;  
      } 
      else {
        if(pauseCounter > 0 ){ 
          pauseCounter--;  
          compress.disable();
        }
        else{ 
          executeCounter = 0;
          pauseCounter = 300; 
        }
      }
    } 
    SmartDashboard.putNumber("Pause Counter", pauseCounter); 
    SmartDashboard.putNumber("Execute Counter", executeCounter);  
    SmartDashboard.putBoolean("Stop Flag", m_Stop);
  }


  public void stop(){ 
    m_Stop = true;
  }
}
