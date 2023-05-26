// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.swervecommands.autoArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw; 


public class AutoClaw extends CommandBase { 
   Claw claw; 
   boolean isFinished = false;

  /** Creates a new AutoClaw. */
  public AutoClaw(Claw c) {
    // Use addRequirements() here to declare subsystem dependencies. 
    claw = c; 

    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if(claw.clawState() == false){ 
      claw.OpenClaw(); 
      isFinished = true;
    } 
    else{ 
      if(claw.clawState() == true){ 
        claw.CloseClaw(); 
        isFinished = true;
      } 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    claw.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
