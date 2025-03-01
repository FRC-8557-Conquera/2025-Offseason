package frc.robot.autos;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class Autonomous extends SequentialCommandGroup {
  public Autonomous(Swerve s_Swerve) {
    
    addCommands(
      new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()),
      new InstantCommand(() -> s_Swerve.zeroGyro()),
      new InstantCommand(() -> s_Swerve.driveCommand(
            () -> 0.0,
            () -> 0.0,
            () -> 0.0))  
     );     
  }
}