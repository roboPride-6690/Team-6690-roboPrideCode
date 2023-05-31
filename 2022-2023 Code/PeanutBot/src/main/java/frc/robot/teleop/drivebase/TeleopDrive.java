// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package frc.robot.teleop.drivebase;

//import frc.robot.Constants.Drivebase;
//import frc.robot.commands.Drive;
//import frc.robot.subsystems.SwerveBase;

//import java.util.function.BooleanSupplier;
//import java.util.function.DoubleSupplier;

//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
//public class TeleopDrive extends CommandBase {
  //private SwerveBase swerve; 
  //private Drive drive;
  //private DoubleSupplier vX, vY, omega;
  //private BooleanSupplier driveMode; 
  //private boolean isOpenLoop; 
  //private double accelerationCounterX;
  //private double accelerationCounterY; 

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  //public TeleopDrive(SwerveBase swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, BooleanSupplier driveMode, boolean isOpenLoop) {
    //this.swerve = swerve;
    //this.vX = vX;
    //this.vY = vY;
    //this.omega = omega;
    //this.driveMode = driveMode;
    //this.isOpenLoop = isOpenLoop; 
    //accelerationCounterX = 0;
    //accelerationCounterY = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(swerve);
  //}

  // Called when the command is initially scheduled.
  //@Override
  //public void initialize() { 
    //accelerationCounterX = 0;
    //accelerationCounterY = 0;
  //}

  // Called every time the scheduler runs while the command is scheduled.
  //@Override
  //public void execute() {
    //double angVelocity = 0; 
    //double xVelocity = 0;
    //double yVelocity = 0;
    
    //slower rotation speed for lower input values. Mr. Sanders 
    //if (omega.getAsDouble() > 0.5){
      //angVelocity = Math.pow(omega.getAsDouble(), 3) * Drivebase.MAX_ANGULAR_VELOCITY;
    //}else{
      //angVelocity = Math.pow(omega.getAsDouble(), 3) * Drivebase.MAX_ANGULAR_VELOCITY/2;
    //} 

    //as long as the user holds toggle the speed will increase
    //if(vY.getAsDouble() > 0.2 || vY.getAsDouble() < -0.2) {
      //if(accelerationCounterY < Math.abs(vY.getAsDouble())){ 
        //accelerationCounterY += 0.002;
      //} 
    //} 
    //else{ 
      //accelerationCounterY = 0;  //reset acceleration back to zero.
    //}

    //yVelocity = Math.pow(vY.getAsDouble(), 3) * -Drivebase.MAX_SPEED * accelerationCounterY;

    //if(vX.getAsDouble() > 0.2 || vX.getAsDouble() < -0.2){ 
      //if(accelerationCounterX < Math.abs(vX.getAsDouble())){ 
        //accelerationCounterX += 0.002;
      //} 
    //} 
    //else{ 
      //accelerationCounterX = 0;
    //}
      
    //xVelocity = Math.pow(vX.getAsDouble(), 3) * Drivebase.MAX_SPEED * accelerationCounterX;

    //SmartDashboard.putNumber("AccY: ", accelerationCounterY);
    //SmartDashboard.putNumber("AccX: ", accelerationCounterX);
    //SmartDashboard.putNumber("vX", xVelocity);
    //SmartDashboard.putNumber("vY", yVelocity);
    //SmartDashboard.putBoolean("field centric", driveMode.getAsBoolean());
    //swerve.drive(
      //new Translation2d(
        //xVelocity,
        //yVelocity),
      //angVelocity,
      //driveMode.getAsBoolean(),
      //isOpenLoop);
  //}

  // Called once the command ends or is interrupted.
  //@Override
  //public void end(boolean interrupted) {}

  // Returns true when the command should end.
  //@Override
  //public boolean isFinished() {
    //return false;
  //}
//}
