package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Peripheral;

public class shooter_al extends Command {
    private final Peripheral shooter;
    public shooter_al(Peripheral shooter){
        this.shooter = shooter;
        addRequirements(shooter);

    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        shooter.shooterIcineal();
    }
    @Override
    public void end(boolean interrupt){
        shooter.shooterDurdur();
    }
    @Override
    public boolean isFinished(){
        shooter.shooterDurdur();
        return false;
    }
    
}
