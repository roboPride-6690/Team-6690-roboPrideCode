// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.teleop.teleopArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist; 


public class TeleopLowerWrist extends CommandBase { 
  private final Wrist wrist; 
  //private final double distance; 

  /** Creates a new ElevateArm. */ 

  public TeleopLowerWrist(Wrist w) {
    // Use addRequirements() here to declare subsystem dependencies. 
    wrist = w;  
    //distance = d; 
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    wrist.turnArm(-Constants.Arm.wristSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  } 

}
