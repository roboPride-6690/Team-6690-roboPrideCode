// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.autonomous.AutonomousOne;
import frc.robot.autonomous.AutonomousTwo;
import frc.robot.subsystems.SwerveBase;
//import frc.robot.teleop.drivebase.TeleopDrive;
//import frc.robot.teleop.teleopArm.TeleopElevateArm;
//import frc.robot.teleop.teleopArm.TeleopLowerArm;
//import frc.robot.teleop.teleopArm.TeleopLowerWrist;
//import frc.robot.teleop.teleopArm.TeleopRaiseWrist;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//import com.revrobotics.RelativeEncoder;

//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveBase drivebase = new SwerveBase();   

 
  // 
  private final AutonomousOne a_Auto;    
  private final AutonomousTwo a_Auto2;
  private static final String kDefaultAuto = "Default"; 
  private static final String kCustomAuto = "My Auto"; 
  private String m_autoSelected; 
  private final SendableChooser<Command> m_Chooser = new SendableChooser<>(); 
  



  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandXboxController driverController = new CommandXboxController(0); 
  private final Joystick driver1 = new Joystick(1);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings 
    configureBindings(); 
    
    //gyro = new ADXRS450_Gyro();
    // AbsoluteDrive absoluteDrive = new AbsoluteDrive(
    //   drivebase,
      // Applies deadbands and inverts controls because joysticks are back-right positive while robot
      // controls are front-left positive
      // () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      // () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      // () -> -rotationController.getX(),
      // () -> -rotationController.getY(), true);

    // AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(
    //   drivebase,
      // Applies deadbands and inverts controls because joysticks are back-right positive while robot
      // // controls are front-left positive
      // () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      // () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      // () -> -rotationController.getX(),
      // () -> -rotationController.getY(), false);

    //TeleopDrive openRobotRel = new TeleopDrive(
     // drivebase,
     // () -> (Math.abs(driverController.getLeftX()) > 0.05) ? -driverController.getLeftX() : 0,
     // () -> (Math.abs(driverController.getLeftY()) > 0.05) ? -driverController.getLeftY() : 0,
     // () -> -driverController.getRightX(), () -> false, true);
    
    //TeleopDrive closedRobotRel = new TeleopDrive(
     // drivebase,
     // () -> (Math.abs(driverController.getLeftY()) > 0.05) ? -driverController.getLeftY() : 0,
     // () -> (Math.abs(driverController.getLeftX()) > 0.05) ? -driverController.getLeftX() : 0,
     // () -> -driverController.getRightX(), () -> true, false);
    
    // TeleopDrive openFieldRel = new TeleopDrive(
    //   drivebase,
    //   () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
    //   () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
    //   () -> -driverController.getTwist(), () -> true, true);

    // TeleopDrive closedFieldRel = new TeleopDrive(
    //   drivebase,
    //   () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
    //   () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
    //   () -> -driverController.getTwist(), () -> true, false);

    //drivebase.setDefaultCommand(closedRobotRel);   
    a_Auto = new AutonomousOne(drivebase); 
    a_Auto2 = new AutonomousTwo(drivebase);
 
    m_Chooser.setDefaultOption("Autonomous 1", a_Auto);  
    m_Chooser.addOption("Autonoumous 2", a_Auto2); 
    SmartDashboard.putData(m_Chooser);
   // registerAutos();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //driverController.button(1).onTrue((new InstantCommand(drivebase::zeroGyro))); 

  }

  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
 
    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_Chooser.getSelected();
  } 
}

