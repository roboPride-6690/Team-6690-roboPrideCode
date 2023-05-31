package frc.robot.autonomous.swervecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveBase;

public class stopDriveMotors extends CommandBase { 
   SwerveBase drive; 
   boolean isFinished = false;

  /** Creates a new AutoClaw. */
  public stopDriveMotors(SwerveBase s) {
    // Use addRequirements() here to declare subsystem dependencies. 
    drive = s; 

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    //drive.autoStop();
    isFinished = true;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
