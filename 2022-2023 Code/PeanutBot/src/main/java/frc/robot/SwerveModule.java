package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;   
import com.ctre.phoenix.motorcontrol.can.TalonSRX; 

import frc.robot.Util.DriveCommand;
import frc.robot.Util.WrappedTalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration; 
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode; 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
//import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.Drivebase;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset, lastAngle;
    private WrappedTalonSRX angleMotor, driveMotor; 
    private Notifier pidLoop;      //A notifier is a thread. Basically think of a thread as something running in the background.
    private volatile double sumError, errorChange, lastError, currentError, pidOutput; 
    private boolean isReversed;
    private double setpoint;
    private double offset;
    private CANCoder absoluteEncoder;
    private RelativeEncoder angleEncoder, driveEncoder;
    private TalonSRXPIDSetConfiguration angleController, driveController;  
    private TalonSRXControlMode controlMode; 

    private static final double dt = 0.02;  //this is how fast we run our PID loop.
    private int kPositiveRotationMin = 45;  //we measured this
    private int kPositiveRotationMax = 870;  //and this

    private int kNegativeRotationMin = 156;  //we measured this
    private int kNegativeRotationMax = 978;  //and this

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Drivebase.KS, Drivebase.KV, Drivebase.KA);

    public SwerveModule(int angleID, int driveID, boolean isReversed, double offset, double kP, double kI, double kD) {
        this.moduleNumber = moduleNumber;
        //angleOffset = moduleConstants.angleOffset;

        angleMotor = new WrappedTalonSRX(angleID);
        driveMotor = new WrappedTalonSRX(driveID);

        angleMotor.reset();
        driveMotor.reset(); 

        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.PeanutConstants.kPIDLoopIdx, Constants.PeanutConstants.kTimeoutMs);
        angleMotor.configOpenloopRamp(0, Constants.PeanutConstants.kTimeoutMs);      //this is what we were missing!
        angleMotor.configPeakCurrentDuration(Constants.PeanutConstants.kPeakCurrentDuration, Constants.PeanutConstants.kTimeoutMs);
        angleMotor.configPeakCurrentLimit(Constants.PeanutConstants.kPeakCurrentLimit, Constants.PeanutConstants.kTimeoutMs);
        angleMotor.configContinuousCurrentLimit(Constants.PeanutConstants.kSustainedCurrentLimit, Constants.PeanutConstants.kTimeoutMs);
        angleMotor.enableCurrentLimit(true);
        angleMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10, 0);
        angleMotor.setInverted(true);
        angleMotor.setSensorPhase(true); 

        //Configure drive Talon SRX
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PeanutConstants.kPIDLoopIdx, Constants.PeanutConstants.kTimeoutMs);

        // Config angle encoders
        //absoluteEncoder = new CANCoder(moduleConstants.cancoderID);
        //absoluteEncoder.configFactoryDefault();
        //CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        //canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        //canCoderConfiguration.sensorDirection = Drivebase.CANCODER_INVERT;
        //canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        //canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
        //absoluteEncoder.configAllSettings(canCoderConfiguration);

        //angleEncoder = angleMotor.getEncoder();
        //angleEncoder.setPositionConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION);
        //angleEncoder.setVelocityConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION / 60);
        //angleEncoder.setPosition(absoluteEncoder.getAbsolutePosition() - angleOffset);

        // Config angle motor/controller
        //angleController = angleMotor.getPIDController();
        //angleController.setP(Drivebase.MODULE_KP); 
        //angleController.setI(Drivebase.MODULE_KI);
        //angleController.setD(Drivebase.MODULE_KD);
        //angleController.setFF(Drivebase.MODULE_KF);
        //angleController.setIZone(Drivebase.MODULE_IZ);
        //angleController.setPositionPIDWrappingEnabled(true);
        //angleController.setPositionPIDWrappingMaxInput(180);
        //angleController.setPositionPIDWrappingMinInput(-180);
        //angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake); 
        //angleMotor.setSmartCurrentLimit(Constants.Drivebase.Angle_Motor_Limit);

        // Config drive motor/controller
        //driveController = driveMotor.getPIDController();
        //driveEncoder = driveMotor.getEncoder();
        //driveEncoder.setPositionConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION);
        //driveEncoder.setVelocityConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION / 60);
        //driveController.setP(Drivebase.VELOCITY_KP);
        //driveController.setI(Drivebase.VELOCITY_KI);
        //driveController.setD(Drivebase.VELOCITY_KD);
        //driveController.setFF(Drivebase.VELOCITY_KF);
        //driveController.setIZone(Drivebase.VELOCITY_IZ);
        //driveMotor.setInverted(Drivebase.DRIVE_MOTOR_INVERT);
        //driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake); 
        //driveMotor.setSmartCurrentLimit(Constants.Drivebase.Drive_Motor_Limit);

        //driveMotor.burnFlash();
        //angleMotor.burnFlash();

        pidLoop = new Notifier(() -> {
            currentError = getModifiedError();  //update the current error to the most recent one
            /*
            sumError += currentError * dt;
            errorChange = (currentError-lastError)/dt;
*/
            pidOutput = kP * currentError; //+ kI * sumError + kD * errorChange; //you guys know this, or at least you better...
            angleMotor.set(ControlMode.PercentOutput, pidOutput);
            //lastError = currentError;   //update the last error to be the current error
        });


        this.isReversed = isReversed;

        pidLoop.startPeriodic(dt);
            
        //lastAngle = getState().angle.getDegrees();
    }
 
    public void setMotor(double speed){ 
        angleEncoder.setPosition(speed);
    }

    //public void setDriveSpeed(double speed){
        //driveMotor.set(null, speed);
    //}

    /**
     * 
     * @return  the unbounded steering error, in radians
     */
    public double getError(){
        return setpoint - getSteeringDegrees();
    }

    /**
     * 
     * @return  the steering error bounded to [-pi, pi]
     */
    public double getModifiedError(){
        return (boundHalfDegrees(getError()))/180;
    } 

        /**
     * 
     * @param power the power of the wheel, where power is [-1.0, 1.0]
     */
    public void setDrivePower(double power){
        if(isReversed)
            driveMotor.set(ControlMode.PercentOutput, -power);
        else
            driveMotor.set(ControlMode.PercentOutput, power);
    }

        /**
     * 
     * @param deg   the angle to set the wheel to, in degrees
     */
    public void setSteeringDegrees(double deg){
        setpoint = boundHalfDegrees(deg + offset);
    } 

        /**
     * 
     * @return  returns the setpoint of the sttering in degrees
     */
    public double getSetpointDegrees(){
        return setpoint;
    }  

    public double getRawSteeringEncoder(){
        return angleMotor.getSelectedSensorPosition(0);
    }

    public double getSpeed(){
        return driveMotor.getSelectedSensorVelocity(0);
    }




    public void configEncValues(int posMin, int posMax, int negMin, int negMax){
        kPositiveRotationMin = posMin;
        kPositiveRotationMax = posMax;

        kNegativeRotationMin = negMin;
        kNegativeRotationMax = negMax;
    }

    public static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }

        /**
     * @return  the angle of the wheel, where angle is an element of [-pi, pi]
     */

     public double getSteeringDegrees(){
        int steeringPosition = (int)angleMotor.getSelectedSensorPosition(Constants.PeanutConstants.kPIDLoopIdx);

        if(steeringPosition >= 0){
            return normalizeEncoder(kPositiveRotationMin, kPositiveRotationMax, steeringPosition)-180;
        }
        else
            return (360-normalizeEncoder(kNegativeRotationMin, kNegativeRotationMax, steeringPosition))-180;
    }

    public double getSteeringDegreesCompensated(){
        return getSteeringDegrees() - offset;
    }
    /**
     * 
     * @return  the closed-loop PID output, calculated by PID loop
     */
    public double getSteeringOutput(){
        return pidOutput;
    }
        /**
     *
     * @param encPos    the encoder input to be normalized
     * @param minVal    the minimum MEASURED ABSOLUTE value of the encoder
     * @param maxVal    the maximum MEASURED ABSOLUTE value of the encoder
     * @return          the encoder input normalized to [0, 1023]
     * */
    private double normalizeEncoder(int minVal, int maxVal, int encPos){
        return ((Math.abs(encPos) % 1023) - minVal) * Math.abs((360.0/(maxVal-minVal)));
    } 

    public void setMotorBrake(boolean brake) {
        driveMotor.set(TalonSRXControlMode.Disabled, 0);

    }

}
