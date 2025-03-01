package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Peripheral;
import frc.robot.Constants.yukari;
public class elevator_ac extends Command {
    private final Peripheral elevator;
    public elevator_ac(Peripheral elevator){
            this.elevator = elevator;
            addRequirements(elevator);
        
    }

    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        elevator.elevatorAc();
        

    }

    @Override
    public void end(boolean interrupt){
        elevator.elevatorDurdur();
    }
    @Override
    public boolean isFinished(){
        elevator.elevatorDurdur();
        return false;
    }
    
    
}
