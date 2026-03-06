package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.ClimberSubsystem;

public class UpToClimb extends Command {
    private ClimberSubsystem climber;

    public UpToClimb(ClimberSubsystem climberSubsystem) {
        this.climber = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climber.MoveDownToClimb();
    }

    @Override
    public boolean isFinished() {
        return climber.MoveUpToClimb();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
    
}
