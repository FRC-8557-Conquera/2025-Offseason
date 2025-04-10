package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MU;
import frc.robot.subsystems.Peripheral;

public class ElevatorCommand extends Command {
    private final Peripheral peripheral;
    private final double target;

    public ElevatorCommand(Peripheral peripheral, double target) {
        this.peripheral = peripheral;
        this.target = target;
    }

    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        double elevatorPos = peripheral.getEncoderPosition();
        if(!MU.isNear(elevatorPos, target, 0.3)) {
            if(elevatorPos > target) {
                peripheral.elevatorKapat();
            } else {
                peripheral.elevatorAc();
            }
        }
    }
    @Override
    public void end(boolean interrupt){
        peripheral.elevatorDurdur();
    }
    @Override
    public boolean isFinished(){
        peripheral.elevatorDurdur();
        return false;
    }
}
