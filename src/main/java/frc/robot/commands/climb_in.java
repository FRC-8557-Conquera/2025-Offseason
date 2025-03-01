package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Peripheral;

public class climb_in extends Command {
    private final Peripheral climb;
    public climb_in(Peripheral climb){
            this.climb = climb;
            addRequirements(climb);
        
    }
    @Override 
    public void initialize(){

    }
    @Override
    public void execute(){
        climb.climbIn();

    }
    @Override
    public void end(boolean interrupt){
        climb.climbDur();
    }
    @Override
    public boolean isFinished(){
        climb.climbDur();
        return false;
    }
    
}
