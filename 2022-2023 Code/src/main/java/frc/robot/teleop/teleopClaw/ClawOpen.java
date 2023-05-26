// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.teleop.teleopClaw;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;
import frc.robot.subsystems.Claw; 
//import frc.robot.subsystems.CompressorSub;

public class ClawOpen extends CommandBase {

    private final Claw claw;  
    private boolean isFinished = false;

    public ClawOpen(Claw c) {
        claw = c;
        isFinished = false;
        //super();
        addRequirements(claw); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        claw.OpenClaw();  
        isFinished = true;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}