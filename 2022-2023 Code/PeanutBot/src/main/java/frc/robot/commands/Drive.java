package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Util.DriveCommand;
import frc.robot.Util.ModuleLocation;

import static frc.robot.Robot.kDrivetrain;
import static frc.robot.Robot.oi;

public class Drive extends CommandBase {
  
  public Drive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(Robot.kDrivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double leftX = oi.getLeftXAxis();
    double leftY = oi.getLeftYAxis();
    double rightX = oi.getRightXAxis();
    double heading = Math.toRadians(Robot.kDrivetrain.getGyro());

    /*
    double temp = leftY*Math.cos(heading) + leftX*Math.sin(heading);
    leftX = -leftY*Math.sin(heading) + leftX*Math.cos(heading);
    leftY = temp;
    */

    double A = leftX - rightX * (Constants.PeanutConstants.kWheelbase/Constants.PeanutConstants.kTrackwidth);
    double B = leftX + rightX * (Constants.PeanutConstants.kWheelbase/Constants.PeanutConstants.kTrackwidth);
    double C = leftY - rightX * (Constants.PeanutConstants.kTrackwidth/Constants.PeanutConstants.kWheelbase);
    double D = leftY + rightX * (Constants.PeanutConstants.kTrackwidth/Constants.PeanutConstants.kWheelbase);

    double frontLeftSpeed = Math.hypot(B, C);
    double frontRightSpeed = Math.hypot(B, D);
    double backLeftSpeed = Math.hypot(A, C);
    double backRightSpeed = Math.hypot(A, D);
    
    double max = frontLeftSpeed;

    if(frontRightSpeed > max)
      max = frontRightSpeed;
    else if(backLeftSpeed > max)
      max = backLeftSpeed;
    else if(backRightSpeed > max)
      max = backRightSpeed;
    if(max > 1){
      frontLeftSpeed /= max;
      frontRightSpeed /= max;
      backLeftSpeed /= max;
      backRightSpeed /= max;
    }

    DriveCommand frontLeftCommand = new DriveCommand((Math.atan2(B, C) * (180.0/Math.PI)), frontLeftSpeed);
    DriveCommand frontRightCommand = new DriveCommand((Math.atan2(B, D) * (180.0/Math.PI)), frontRightSpeed);
    DriveCommand backLeftCommand = new DriveCommand((Math.atan2(A, C) * (180.0/Math.PI) ), backLeftSpeed);
    DriveCommand backRightCommand = new DriveCommand((Math.atan2(A, D) * (180.0/Math.PI)), backRightSpeed);

    kDrivetrain.setModule(ModuleLocation.FRONT_LEFT, frontLeftCommand);
    kDrivetrain.setModule(ModuleLocation.FRONT_RIGHT, frontRightCommand);
    kDrivetrain.setModule(ModuleLocation.BACK_LEFT, backLeftCommand);
    kDrivetrain.setModule(ModuleLocation.BACK_RIGHT, backRightCommand);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  } 
}
