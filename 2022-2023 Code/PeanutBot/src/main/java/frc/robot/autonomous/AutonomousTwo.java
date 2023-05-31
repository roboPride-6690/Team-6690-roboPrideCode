package frc.robot.autonomous;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.autonomous.swervecommands.autoSwerve.SwerveTrajectory;
import frc.robot.autonomous.swervecommands.stopDriveMotors;
import frc.robot.autonomous.swervecommands.autoSwerve.exampleAuto;
//Subsystem Imports 
import frc.robot.subsystems.SwerveBase;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro; 
//import frc.robot.autonomous.ParallelCommandsOne;

public class AutonomousTwo extends SequentialCommandGroup { 

    /** Creates a new AutonomousOne. */ 
    //SwerveBase s_SwerveBase; 
    public AutonomousTwo(SwerveBase s) {
      //Use addRequirements() here to declare subsystem dependencies.   

      //Position 1(Starting Position Is On The Left Side Of The Field) 

      addCommands(Commands.sequence(new exampleAuto(s, 0, 0, 0, 1.95, 0, 0)));    
      addCommands(Commands.sequence(new stopDriveMotors(s)));   
      //addCommands(Commands.sequence(Commands.parallel(new ClawCollect(c))), new AutoElevateArm(sh, -1, e));   
      //addCommands(Commands.sequence(new stopDriveMotors(s)));  
      addCommands(Commands.sequence(new exampleAuto(s, 0, 0, 0, 0, 1, 3.14 *.5))); 
      addCommands(Commands.sequence(new exampleAuto(s, 0, 0, 0,  3, 0, Math.PI * 0.5)));
      addCommands(Commands.sequence(new exampleAuto(s, 0, 0, 0, 3.45, 0, 0)));    
    
      //addCommands(Commands.sequence(new exampleAuto(s, 0, 0, 0,  1, 0, 0.0)));    
      //addCommands(Commands.sequence(new exampleAuto(s, 0, 0, 0, -5.98, 0, 3.14)));    
      //addCommands(Commands.sequence(new AutoElevateArm(sh, 1, e)));
      //addCommands(Commands.sequence(new AutoElevateArm(sh, -1, e)));
    }
  }