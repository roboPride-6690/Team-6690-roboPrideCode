// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.teleop.teleopArm;

//import edu.wpi.first.wpilibj.Encoder;  
import com.revrobotics.RelativeEncoder;
//import edu.wpi.first.wpilibj.XboxController; 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
//import frc.robot.RobotContainer;
import frc.robot.subsystems.Shoulder; 

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevateArmWithJoysticks extends CommandBase {
  /** Creates a new DriveForwardWithJoysticks. */
  public static Joystick joystick; 
  public static JoystickButton buttonNumber;  
  private final Shoulder arm;
  private final RelativeEncoder encoder;
  
  public ElevateArmWithJoysticks(Shoulder a, RelativeEncoder e, Joystick j) {
    // Use addRequirements() here to declare subsystem dependencies.
    joystick = j; 
    buttonNumber = new JoystickButton(j, Constants.Joystick.BUTTON_Number); 
    arm = a; 
    encoder = e; 
     
    addRequirements(arm);
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
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
