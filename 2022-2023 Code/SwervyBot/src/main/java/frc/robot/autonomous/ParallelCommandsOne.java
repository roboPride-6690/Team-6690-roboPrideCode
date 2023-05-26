package frc.robot.autonomous;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.autonomous.swervecommands.autoArm.AutoElevateArm;
import frc.robot.autonomous.swervecommands.autoArm.AutoWrist;
import frc.robot.autonomous.swervecommands.autoSwerve.exampleAuto;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.Wrist;

public class ParallelCommandsOne extends ParallelCommandGroup{
    public ParallelCommandsOne(SwerveBase s, Shoulder sh, RelativeEncoder e, Wrist w){ 
        addCommands(
            new AutoWrist(w, 1, e), 
            new AutoElevateArm(sh, 1, e)
            );
    } 
}
