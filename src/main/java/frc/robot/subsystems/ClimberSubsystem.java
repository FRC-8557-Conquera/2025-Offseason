package frc.robot.subsystems;

import java.util.concurrent.ConcurrentLinkedDeque;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    public static final SparkMax climb = new SparkMax(Constants.ClimberConstants.climbID, SparkLowLevel.MotorType.kBrushless);

    
    public void climbPress(){
        climb.set(0.9);
    }
    public void climbWithdraw(){
        climb.set(-0.9);
    }
    public void climbStop(){
        climb.set(0);
    }
}
