package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double target;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double target) {
        this.elevator = elevatorSubsystem;
        this.target = target;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        elevator.setElevatorHeight(target);
    }
    @Override
    public void end(boolean interrupt){
        elevator.stop();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
