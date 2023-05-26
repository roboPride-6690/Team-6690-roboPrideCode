package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants; 

public class Wrist extends SubsystemBase{
    private CANSparkMax wristMotor;
    private Joystick controller;
    private double WristSpeed; 
    //private boolean initialized = false;  

    public Wrist(Joystick c, double s) {     
        
        //Wrist Motor Configuration 
        //wristMotor = new CANSparkMax(Constants.Arm.wristID, MotorType.kBrushless); 
        wristMotor.restoreFactoryDefaults();  
        wristMotor.setSmartCurrentLimit(Constants.Arm.Wrist_Motor_Limit);
        wristMotor.setIdleMode(IdleMode.kBrake); 
        wristMotor.enableVoltageCompensation(Constants.Arm.voltageComp);
        
        controller = c;
        WristSpeed = s;
    }

  //Moving Wrist With Z Rotation Joystick
  public void moveWristWithJoysticks(){ 
    SmartDashboard.putNumber("Moving Wrist: ", -WristSpeed*controller.getZ());
    SmartDashboard.putNumber("Joystick getZ: ", controller.getZ());
    if (controller.getZ() > 0.3 || controller.getZ() < -0.3){
      wristMotor.set(WristSpeed*controller.getZ());
    }
    else {
      wristMotor.set(0);
    }
  }

    public void turnArm(double speed){
        wristMotor.set(-speed);
    }

    public RelativeEncoder getWristEncoder(){ 
        return wristMotor.getEncoder();
    }

    public void stop(){
        wristMotor.stopMotor();
    }     

    @Override
    public void periodic() {
      // This method will be called once per scheduler run   
      moveWristWithJoysticks();
      SmartDashboard.putNumber("Wrist Position", wristMotor.getEncoder().getPosition());   

      //if(initialized == false){ 
        //if (wristMotor.getEncoder().getPosition() < -10){
        //  initialized = true; 
        //}
        //else {
         //wristMotor.set(-Constants.Arm.wristSpeed);
        //} 
        //} 
      //} 
    }
}