// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.swervecommands.autoArm;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist; 


public class AutoWrist extends CommandBase { 
  Wrist wrist;  
  RelativeEncoder encoder;
  double angleDistance; 
  boolean reverseMode;
  double timeCounter = 0;
  boolean isFinished = false;

  /** Creates a new ElevateArm. */ 

  public AutoWrist(Wrist w, double an, RelativeEncoder e) {
    // Use addRequirements() here to declare subsystem dependencies. 
    wrist = w;   
    timeCounter = 0;
    if(an < 0){ 
      reverseMode = true; 
    }
    angleDistance = an;  
    encoder = e; 
    addRequirements(wrist);

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

    if (timeCounter > 117.5){
    //if (encoder.getPosition() > 6){
      isFinished = true;
    }
    else {
      if (reverseMode){
        wrist.turnArm(Constants.Arm.auto_wristSpeed);
        timeCounter ++;
      }
      else{
          wrist.turnArm(-Constants.Arm.auto_wristSpeed);
          timeCounter ++;
      }
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  } 
}
