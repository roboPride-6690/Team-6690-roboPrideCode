// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.DriveCommand;
import frc.robot.Util.ModuleLocation;
import frc.robot.commands.Drive;
import frc.robot.SwerveModule;

public class SwerveBase extends SubsystemBase {
  private SwerveModule moduleFrontLeft = new SwerveModule(Constants.PeanutConstants.kFrontLeftSteerID, Constants.PeanutConstants.kFrontLeftDriveID, true, Constants.PeanutConstants.kFrontLeftOffset, Constants.PeanutConstants.kSwerveP, Constants.PeanutConstants.kSwerveI, Constants.PeanutConstants.kSwerveD);
  private SwerveModule moduleFrontRight = new SwerveModule(Constants.PeanutConstants.kFrontRightSteerID, Constants.PeanutConstants.kFrontRightDriveID, true, Constants.PeanutConstants.kFrontRightOffset, Constants.PeanutConstants.kSwerveP, Constants.PeanutConstants.kSwerveI, Constants.PeanutConstants.kSwerveD);
  private SwerveModule moduleBackLeft = new SwerveModule(Constants.PeanutConstants.kBackLeftSteerID, Constants.PeanutConstants.kBackLeftDriveID, true, Constants.PeanutConstants.kBackLeftOffset, Constants.PeanutConstants.kSwerveP, Constants.PeanutConstants.kSwerveI, Constants.PeanutConstants.kSwerveD);
  private SwerveModule moduleBackRight = new SwerveModule(Constants.PeanutConstants.kBackRightSteerID, Constants.PeanutConstants.kBackRightDriveID, true, Constants.PeanutConstants.kBackRightOffset, Constants.PeanutConstants.kSwerveP, Constants.PeanutConstants.kSwerveI, Constants.PeanutConstants.kSwerveD);
  public SwerveBase(){
    moduleFrontLeft.configEncValues(47, 855, 156, 978);
    moduleFrontRight.configEncValues(44, 853, 156, 978);
    moduleBackLeft.configEncValues(45, 868, 156, 978);  
    moduleBackRight.configEncValues(43, 865, 156, 978);
  }
  
  public void initDefaultCommand() {
    setDefaultCommand(new Drive());
  }
  
  public SwerveModule getModule(ModuleLocation location){
    switch(location){
      case FRONT_LEFT:
        return moduleFrontLeft;
      case FRONT_RIGHT:
        return moduleFrontRight;
      case BACK_LEFT:
        return moduleBackLeft;
      case BACK_RIGHT:
        return moduleBackRight;
      default:
        return null;
    }
  }

  public void setModule(ModuleLocation loc, double degrees, double power){
    switch(loc){
      case FRONT_LEFT:
        moduleFrontLeft.setDrivePower(power);
        moduleFrontLeft.setSteeringDegrees(degrees);
        break;
      case FRONT_RIGHT:
      moduleFrontRight.setDrivePower(power);
      moduleFrontRight.setSteeringDegrees(degrees);
        break;
      case BACK_LEFT:
      moduleBackLeft.setDrivePower(power);
      moduleBackLeft.setSteeringDegrees(degrees);
        break;
      case BACK_RIGHT:
      moduleBackRight.setDrivePower(power);
      moduleBackRight.setSteeringDegrees(degrees);
        break;
    }
  }

  public void setModule(ModuleLocation loc, DriveCommand command){
    setModule(loc, command.getDegrees(), command.getSpeed());
  }

  public void setAllAngle(double degrees){
    moduleFrontLeft.setSteeringDegrees(degrees);
    moduleFrontRight.setSteeringDegrees(degrees);
    moduleBackLeft.setSteeringDegrees(degrees);
    moduleBackRight.setSteeringDegrees(degrees);
  }

  public void setAllPower(double power){
    moduleFrontLeft.setDrivePower(power);
    moduleFrontRight.setDrivePower(power);
    moduleBackLeft.setDrivePower(power);
    moduleBackRight.setDrivePower(power);
  }

  public void updateDashboard(){
    SmartDashboard.putNumber("Front Left Error", moduleFrontLeft.getError());
    SmartDashboard.putNumber("Front Right Error", moduleFrontRight.getError());
    SmartDashboard.putNumber("Back Left Error", moduleBackLeft.getError());
    SmartDashboard.putNumber("Back Right Error", moduleBackRight.getError());

    SmartDashboard.putNumber("Front Left Setpoint", moduleFrontLeft.getSetpointDegrees());
    SmartDashboard.putNumber("Front Right Setpoint", moduleFrontRight.getSetpointDegrees());
    SmartDashboard.putNumber("Back Left Setpoint", moduleBackLeft.getSetpointDegrees());
    SmartDashboard.putNumber("Back Right Setpoint", moduleBackRight.getSetpointDegrees());

    SmartDashboard.putNumber("Front Left Position", moduleFrontLeft.getSteeringDegrees());
    SmartDashboard.putNumber("Front Right Position", moduleFrontRight.getSteeringDegrees());
    SmartDashboard.putNumber("Back Left Position", moduleBackLeft.getSteeringDegrees());
    SmartDashboard.putNumber("Back Right Position", moduleBackRight.getSteeringDegrees());

    SmartDashboard.putNumber("Front Left Raw Position", moduleFrontLeft.getRawSteeringEncoder());
    SmartDashboard.putNumber("Front Right Raw Position", moduleFrontRight.getRawSteeringEncoder());
    SmartDashboard.putNumber("Back Left Raw Position", moduleBackLeft.getRawSteeringEncoder());
    SmartDashboard.putNumber("Back Right Raw Position", moduleBackRight.getRawSteeringEncoder());

    SmartDashboard.putNumber("Front Left Speed", moduleFrontLeft.getSpeed());
    SmartDashboard.putNumber("Front Right Speed", moduleFrontRight.getSpeed());
    SmartDashboard.putNumber("Back Left Speed", moduleBackLeft.getSpeed());
    SmartDashboard.putNumber("Back Right Speed", moduleBackRight.getSpeed());
  }

  public double getGyro(){
    return 0;
  } 

  /**
   * Sets the drive motors to brake/coast mode.
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    moduleFrontLeft.setMotorBrake(brake); 
    moduleFrontRight.setMotorBrake(brake);  
    moduleBackLeft.setMotorBrake(brake);  
    moduleBackRight.setMotorBrake(brake); 
  }
}