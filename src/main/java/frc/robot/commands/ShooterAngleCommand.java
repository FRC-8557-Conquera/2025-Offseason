package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MU;
import frc.robot.subsystems.Peripheral;

public class ShooterAngleCommand extends Command {
    private final Peripheral peripheral;
    private final double target;

    public ShooterAngleCommand(Peripheral peripheral, double target) {
        this.peripheral = peripheral;
        this.target = target;
        addRequirements(peripheral);
    }
    @Override
    public void initialize(){

    }
    @Override
    // TODO: Açı yönünün doğru olup olmadığına bak
    public void execute(){
        double angle = peripheral.getEncoderAciPosition();

        if(!MU.isNear(angle, target, 10)) {
            if(angle > target) {
                peripheral.ShooteraciAsagi();
            } else {
                peripheral.ShooteraciYukari();
            }
        }
    }
    @Override
    public void end(boolean interrupt){
        peripheral.ShooteraciDurdur();
    }
    @Override
    public boolean isFinished(){
        return MU.isNear(peripheral.getEncoderAciPosition(), target, 10);
    }
}
