// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.teleop.teleopWrist;

//import edu.wpi.first.wpilibj.Encoder;  
import com.revrobotics.RelativeEncoder;
//import edu.wpi.first.wpilibj.XboxController; 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
//import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist;  


//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MoveWristWithJoysticks extends CommandBase {
  /** Creates a new DriveForwardWithJoysticks. */
  public static Joystick joystick; 
  public static JoystickButton buttonNumber;  
  private final Wrist wrist;
  private final RelativeEncoder encoder;
  
  public MoveWristWithJoysticks(Wrist a, RelativeEncoder e, Joystick j) {
    // Use addRequirements() here to declare subsystem dependencies.
    joystick = j; 
    buttonNumber = new JoystickButton(j, Constants.Joystick.BUTTON_Number); 
    wrist = a; 
    encoder = e; 
     
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoder.setPosition(0); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
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
