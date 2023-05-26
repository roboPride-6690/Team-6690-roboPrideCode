// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.swervecommands.autoArm;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shoulder; 


public class AutoElevateArm extends CommandBase { 
  Shoulder arm;  
  RelativeEncoder encoder;
  double angleDistance; 
  boolean reverseMode;
  boolean isFinished = false;

  /** Creates a new ElevateArm. */ 

  public AutoElevateArm(Shoulder a, double an, RelativeEncoder e) {
    // Use addRequirements() here to declare subsystem dependencies. 
    arm = a;   
    if(an < 0){ 
      reverseMode = true; 
    }
    angleDistance = an;  
    encoder = e; 
    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    encoder.setPosition(0); 
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 

    if ( reverseMode){
        if (encoder.getPosition() < 83){
          arm.elevate(Constants.Arm.shoulderSpeed);
        }
        else {
          isFinished = true;
        }
    }
    else {
        if (encoder.getPosition() < -70){
          isFinished = true;
        }
        else {
          arm.elevate(-Constants.Arm.shoulderSpeed);
        }
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  } 
}
