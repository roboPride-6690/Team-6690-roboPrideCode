package frc.robot.autonomous;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.swervecommands.stopDriveMotors;
import frc.robot.autonomous.swervecommands.autoArm.AutoClaw;
import frc.robot.autonomous.swervecommands.autoArm.AutoElevateArm;
import frc.robot.autonomous.swervecommands.autoArm.AutoWrist;
import frc.robot.autonomous.swervecommands.autoSwerve.autoBalance;
import frc.robot.autonomous.swervecommands.autoSwerve.exampleAuto;
import frc.robot.autonomous.swervecommands.autoSwerve.exampleAuto2;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Claw;
//Subsystem Imports 
import frc.robot.subsystems.SwerveBase; 
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.subsystems.Wrist;

public class AutonomousTwo extends SequentialCommandGroup {
    /** Creates a new AutonomousOne. */ 
    //SwerveBase s_SwerveBase;  

    //Use this autonomous to balance on the charge station
    public AutonomousTwo(SwerveBase s, Shoulder a, RelativeEncoder e, Claw c, Wrist w) {
      //Use addRequirements() here to declare subsystem dependencies.   
        addCommands((new autoBalance(s, 0, 0, 0, 2.33, 0, 0))); //2.33 
        addCommands(Commands.sequence(new stopDriveMotors(s))); 
        addCommands((new exampleAuto2(s, 0, 0, 0, 0, 0.1, 0)));   
        addCommands(Commands.sequence(new stopDriveMotors(s))); 


        //addCommands(Commands.sequence(new AutoWrist(w, 1, e)));
    }
  }