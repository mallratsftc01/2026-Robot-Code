package frc.robot.subsystems.topdeck;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.BeamBreakConstants.*;

public class BeamBreak extends SubsystemBase {
    private final DigitalInput beam1;
    private final DigitalInput beam2;

    public BeamBreak() {
        beam1 = new DigitalInput(BEAM_BREAK1);
        beam2 = new DigitalInput(3);
    }

    public boolean getShooter() {
        return beam1.get();
    }

    public boolean getHopper() {
        return beam2.get();
    }

    public boolean getEither() {
        return beam1.get() || beam2.get();
    }

    public boolean getBoth() {
        return beam1.get() && beam2.get();
    }

}
