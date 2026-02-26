package frc.robot.commands.shooter.advancer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;

public class Climb extends Command {
    private AdvancerSubsystem a;
    private DoubleSupplier joy;

    public Climb(AdvancerSubsystem ad, DoubleSupplier yo) {
        a = ad;
        joy = yo;
        addRequirements(a);
    }

    @Override
    public void execute() {
        a.climber(joy.getAsDouble());
    }
}
