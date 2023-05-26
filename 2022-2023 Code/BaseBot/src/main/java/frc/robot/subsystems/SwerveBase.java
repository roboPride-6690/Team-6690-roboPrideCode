// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS; 
import edu.wpi.first.wpilibj.ADXRS450_Gyro;  

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;  
import edu.wpi.first.wpilibj.PWM;  
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d; 
//import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.Constants.Drivebase;

public class SwerveBase extends SubsystemBase {

  private SwerveModule[] swerveModules;
  // private PigeonIMU imu;
  private AHRS imu;
  //private ADXRS450_Gyro gyro; 
  private PWM pwm; 
  
  private SwerveDriveOdometry odometry;
  public Field2d field = new Field2d();

  private boolean wasGyroReset; 
  private boolean enabled;

  /** Creates a new swerve drivebase subsystem.  Robot is controlled via the drive() method,
   * or via the setModuleStates() method.  The drive() method incorporates kinematics— it takes a 
   * translation and rotation, as well as parameters for field-centric and closed-loop velocity control.
   * setModuleStates() takes a list of SwerveModuleStates and directly passes them to the modules.
   * This subsytem also handles odometry.
  */
  public SwerveBase() {

    // imu = new PigeonIMU(0);
    // imu.configFactoryDefault();  

    //gyro = new ADXRS450_Gyro(); 
    pwm = new PWM(0);

    try{ 
      imu = new AHRS(SerialPort.Port.kMXP); 
      imu.enableLogging(true);
    }catch(RuntimeException ex){ 
      DriverStation.reportError("Error instantiating navX MXP" + ex.getMessage(), true);
    }


    this.swerveModules = new SwerveModule[] {
      new SwerveModule(1, Drivebase.Mod1.CONSTANTS), // FRONT Right
      new SwerveModule(0, Drivebase.Mod0.CONSTANTS), // FRONT Left 
      new SwerveModule(3, Drivebase.Mod3.CONSTANTS), // BACK Right
      new SwerveModule(2, Drivebase.Mod2.CONSTANTS), // BACK Left
    };

    odometry = new SwerveDriveOdometry(Drivebase.KINEMATICS, getYaw(), getModulePositions());
    zeroGyro(); 
    resetMods(1); 

    SmartDashboard.putData("Field", field); 
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }

  /**
   * The primary method for controlling the drivebase.  Takes a Translation2d and a rotation rate, and 
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity
   * control for the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation
   *  vector is used.
   * @param translation  Translation2d that is the commanded linear velocity of the robot, in meters per second.
   * In robot-relative mode, positive x is torwards the bow (front) and positive y is torwards port (left).  In field-
   * relative mode, positive x is away from the alliance wall (field North) and positive y is torwards the left wall when looking 
   * through the driver station glass (field West).
   * @param rotation  Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot relativity.
   * @param fieldRelative  Drive mode.  True for field-relative, false for robot-relative.
   * @param isOpenLoop  Whether or not to use closed-loop velocity control.  Set to true to disable closed-loop.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // Creates a robot-relative ChassisSpeeds object, converting from field-relative speeds if necessary.
    ChassisSpeeds velocity = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
      translation.getX(), 
      translation.getY(), 
      rotation, 
      getYaw()
    )
    : new ChassisSpeeds(
      translation.getX(),
      translation.getY(),
      rotation
    ); 

    // Display commanded speed for testing
    SmartDashboard.putString("RobotVelocity", velocity.toString());

    // Calculate required module states via kinematics
    SwerveModuleState[] swerveModuleStates = 
      Drivebase.KINEMATICS.toSwerveModuleStates(
        velocity
      );

    // Desaturate calculated speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Drivebase.MAX_SPEED);

    // Command and display desired states
    for (SwerveModule module : swerveModules) {
      SmartDashboard.putString("Module" + module.toString(), swerveModuleStates[module.moduleNumber].toString());
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
    }
  }

  public void autoDrive(double x, double y, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // Creates a robot-relative ChassisSpeeds object, converting from field-relative speeds if necessary.
    ChassisSpeeds velocity = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
      x, 
      y, 
      rotation, 
      getYaw()
    )
    : new ChassisSpeeds(
      x,
      y,
      rotation
    ); 

    // Display commanded speed for testing
    SmartDashboard.putString("RobotVelocity", velocity.toString());

    // Calculate required module states via kinematics
    SwerveModuleState[] swerveModuleStates = 
      Drivebase.KINEMATICS.toSwerveModuleStates(
        velocity
      );

    // Desaturate calculated speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Drivebase.MAX_SPEED);

    // Command and display desired states
    for (SwerveModule module : swerveModules) {
      SmartDashboard.putString("Module" + module.toString(), swerveModuleStates[module.moduleNumber].toString());
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Set the module states (azimuth and velocity) directly.  Used primarily for auto
   * pathing.
   * @param desiredStates  A list of SwerveModuleStates to send to the modules.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    
    // Desaturates wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Drivebase.MAX_SPEED);

    // Sets states
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.moduleNumber], false);
    }
  } 

  public void operatorControl() {
    while (isOperatorControl() && isEnabled()) {
        
        Timer.delay(0.020);		/* wait for one motor update time period (50Hz)     */
        
        //boolean zero_yaw_pressed = stick.getTrigger();
        //if ( zero_yaw_pressed ) {
        //    imu.zeroYaw();
        //}

        /* Display 6-axis Processed Angle Data                                      */
        SmartDashboard.putBoolean(  "IMU_Connected",        imu.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    imu.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              imu.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
        
        /* Display tilt-corrected, Magnetometer-based heading (requires             */
        /* magnetometer calibration to be useful)                                   */
        
        //SmartDashboard.putNumber(   "IMU_CompassHeading",   imu.getCompassHeading());
        
        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
        //SmartDashboard.putNumber(   "IMU_FusedHeading",     imu.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
        
        //SmartDashboard.putNumber(   "IMU_TotalYaw",         imu.getAngle());
        //SmartDashboard.putNumber(   "IMU_YawRateDPS",       imu.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        
        //SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
        //SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
        //SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());
        //SmartDashboard.putBoolean(  "IMU_IsRotating",       imu.isRotating());

        /* Display estimates of velocity/displacement.  Note that these values are  */
        /* not expected to be accurate enough for estimating robot position on a    */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially      */
        /* double (displacement) integration.                                       */
        
        //SmartDashboard.putNumber(   "Velocity_X",           imu.getVelocityX());
        //SmartDashboard.putNumber(   "Velocity_Y",           imu.getVelocityY());
        //SmartDashboard.putNumber(   "Displacement_X",       imu.getDisplacementX());
        //SmartDashboard.putNumber(   "Displacement_Y",       imu.getDisplacementY());
        
        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
        /* NOTE:  These values are not normally necessary, but are made available   */
        /* for advanced users.  Before using this data, please consider whether     */
        /* the processed data (see above) will suit your needs.                     */
        
        SmartDashboard.putNumber(   "RawGyro_X",            imu.getRawGyroX());
        SmartDashboard.putNumber(   "RawGyro_Y",            imu.getRawGyroY());
        SmartDashboard.putNumber(   "RawGyro_Z",            imu.getRawGyroZ());
        SmartDashboard.putNumber(   "RawAccel_X",           imu.getRawAccelX());
        SmartDashboard.putNumber(   "RawAccel_Y",           imu.getRawAccelY());
        SmartDashboard.putNumber(   "RawAccel_Z",           imu.getRawAccelZ());
        SmartDashboard.putNumber(   "RawMag_X",             imu.getRawMagX());
        SmartDashboard.putNumber(   "RawMag_Y",             imu.getRawMagY());
        SmartDashboard.putNumber(   "RawMag_Z",             imu.getRawMagZ());
        SmartDashboard.putNumber(   "IMU_Temp_C",           imu.getTempC());
        
        /* Omnimount Yaw Axis Information                                           */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
        AHRS.BoardYawAxis yaw_axis = imu.getBoardYawAxis();
        SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
        
        /* Sensor Board Information                                                 */
        SmartDashboard.putString(   "FirmwareVersion",      imu.getFirmwareVersion());
        
        /* Quaternion Data                                                          */
        /* Quaternions are fascinating, and are the most compact representation of  */
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
        /* from the Quaternions.  If interested in motion processing, knowledge of  */
        /* Quaternions is highly recommended.                                       */
        //SmartDashboard.putNumber(   "QuaternionW",          imu.getQuaternionW());
        //SmartDashboard.putNumber(   "QuaternionX",          imu.getQuaternionX());
        //SmartDashboard.putNumber(   "QuaternionY",          imu.getQuaternionY());
        //SmartDashboard.putNumber(   "QuaternionZ",          imu.getQuaternionZ());
        
        /* Connectivity Debugging Support                                           */
        //SmartDashboard.putNumber(   "IMU_Byte_Count",       imu.getByteCount());
        //SmartDashboard.putNumber(   "IMU_Update_Count",     imu.getUpdateCount());
    }
}

  
  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
    // but not the reverse.  However, because this transform is a simple rotation, negating the angle
    // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
    return ChassisSpeeds.fromFieldRelativeSpeeds(Drivebase.KINEMATICS.toChassisSpeeds(getStates()), getYaw().unaryMinus());
  }

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return Drivebase.KINEMATICS.toChassisSpeeds(getStates());
  }

  
  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to 
   * be reset when calling this method.  However, if either gyro angle or module position
   * is reset, this must be called in order for odometry to keep working.
   * @param pose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  /**
   * Gets the current module states (azimuth and velocity)
   * @return A list of SwerveModuleStates containing the current module states
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[Drivebase.NUM_MODULES];
    for (SwerveModule module : swerveModules) {
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  /**
   * Gets the current module positions (azimuth and wheel position (meters))
   * @return A list of SwerveModulePositions cointaing the current module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[Drivebase.NUM_MODULES];
    for (SwerveModule module : swerveModules) {
      positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }

  /**
   * A public method to allow other systems to determine if the gyro was reset by accessing
   * the wasGyroReset flag.
   * @return The boolean value of wasGyroReset
   */
  public boolean wasGyroReset() {
    return wasGyroReset;
  }

  /**
   * Sets wasGyroReset to false.  Should be called after all systems that need to know have called
   * wasGyroReset.
   */
  public void clearGyroReset() {
    wasGyroReset = false;
  } 

  public synchronized boolean isEnabled() {
    return this.enabled;
  } 

  public boolean isOperatorControl(){
    return DriverStation.isTeleopEnabled();
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   * Also sets the wasGyroReset flag to true.
   */
  public void zeroGyro() {
    // Resets the real gyro or the angle accumulator, depending on whether the robot is being simulated
    // imu.setYaw(0);
    imu.reset();
    wasGyroReset = true;
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
   * @return The yaw angle
   */
  public Rotation2d getYaw() {
    //double[] ypr = new double[3];
    // imu.getYawPitchRoll(ypr); 
    //SmartDashboard.putBoolean("is gyro connected?: ", gyro.isConnected());
    //SmartDashboard.putNumber("gyro angle?: ", gyro.getAngle()); 
    SmartDashboard.putBoolean("is navX connected?: ", imu.isConnected());
    SmartDashboard.putNumber("navX angle?: ", imu.getAngle()); 
    SmartDashboard.putNumber("navX z-axis?: ", imu.getRawGyroZ()); 
    SmartDashboard.putNumber("navX y-axis?: ", imu.getRawGyroY());  
    SmartDashboard.putNumber("navX x-axis?: ", imu.getRawGyroX());  
    SmartDashboard.putNumber("navX altitude?: ", imu.getYaw());
   // SmartDashboard.putBoolean("is gyro cali?: ", gyro.  .isCalibrating()); 
    return (Drivebase.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - imu.getAngle()) : Rotation2d.fromDegrees(imu.getAngle());
  } 



  public void resetMods(double speed){ 
    double angleDiff = 0; 

    //For Mod 0
    if(swerveModules[1].getCANCoder() > Constants.Drivebase.Mod0.ANGLE_OFFSET){ 
      angleDiff = swerveModules[1].getCANCoder() - Constants.Drivebase.Mod0.ANGLE_OFFSET;
      swerveModules[1].setMotor(-speed*angleDiff);
    }
    if(swerveModules[1].getCANCoder() < Constants.Drivebase.Mod0.ANGLE_OFFSET){ 
      angleDiff = swerveModules[1].getCANCoder() - Constants.Drivebase.Mod0.ANGLE_OFFSET;
      swerveModules[1].setMotor(-speed*angleDiff);
    }

    //For Mod 1
    if(swerveModules[0].getCANCoder() > Constants.Drivebase.Mod1.ANGLE_OFFSET){ 
      angleDiff = swerveModules[0].getCANCoder() - Constants.Drivebase.Mod1.ANGLE_OFFSET;
      swerveModules[0].setMotor(-speed*angleDiff);
    }
    if(swerveModules[0].getCANCoder() < Constants.Drivebase.Mod1.ANGLE_OFFSET){ 
      angleDiff = swerveModules[0].getCANCoder() - Constants.Drivebase.Mod1.ANGLE_OFFSET;
      swerveModules[0].setMotor(-speed*angleDiff);
    }
      //For Mod 2
      if(swerveModules[3].getCANCoder() > Constants.Drivebase.Mod2.ANGLE_OFFSET){ 
        angleDiff = swerveModules[3].getCANCoder() - Constants.Drivebase.Mod2.ANGLE_OFFSET;
        swerveModules[3].setMotor(-speed*angleDiff);
      }
      if(swerveModules[3].getCANCoder() < Constants.Drivebase.Mod2.ANGLE_OFFSET){ 
        angleDiff = swerveModules[3].getCANCoder() - Constants.Drivebase.Mod2.ANGLE_OFFSET;
        swerveModules[3].setMotor(-speed*angleDiff);
      }
      //For Mod 3
      if(swerveModules[2].getCANCoder() > Constants.Drivebase.Mod3.ANGLE_OFFSET){ 
        angleDiff = swerveModules[2].getCANCoder() - Constants.Drivebase.Mod3.ANGLE_OFFSET;
        swerveModules[2].setMotor(-speed*angleDiff);
      }
      if(swerveModules[2].getCANCoder() < Constants.Drivebase.Mod3.ANGLE_OFFSET){ 
        angleDiff = swerveModules[2].getCANCoder() - Constants.Drivebase.Mod3.ANGLE_OFFSET;
        swerveModules[2].setMotor(-speed*angleDiff);
      }
    }


  /**
   * Sets the drive motors to brake/coast mode.
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setMotorBrake(brake);
    }
  }

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move.
   */
  public void setDriveBrake() {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setDesiredState(
        new SwerveModuleState(
          0,
          Drivebase.MODULE_LOCATIONS[swerveModule.moduleNumber].getAngle()),
        true);
    }
  }

  public void autoStop(){
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setDriveSpeed(0);
    }
  }

  @Override
  public void periodic() {
    // Update odometry
    odometry.update(getYaw(), getModulePositions());

    for (SwerveModule module : swerveModules) {
      SmartDashboard.putNumber("Module" + module.moduleNumber + "CANCoder", module.getCANCoder());
      SmartDashboard.putNumber("Module" + module.moduleNumber + "Relative Encoder", module.getRelativeEncoder());
    }
  }  

  @Override
  public void simulationPeriodic() {
  }
}
