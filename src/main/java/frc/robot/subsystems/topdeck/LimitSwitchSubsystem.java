package frc.robot.subsystems.topdeck;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LimitSwitchConstants.*;

public class LimitSwitchSubsystem extends SubsystemBase {
    
    private final DigitalInput limit1;
    private final DigitalInput limit2;

    public LimitSwitchSubsystem() {
        limit1 = new DigitalInput(9);
        limit2 = new DigitalInput(LIMIT_SWITCH2_PORT);
    }

    public boolean getClimber() {
        return limit1.get();
    }

    public boolean getLimit2() {
        return limit2.get();
    }

    public boolean getEitherLimit() {
        return limit1.get() || limit2.get();
    }

    public boolean getBothLimits() {
        return limit1.get() && limit2.get();
    }
    
}
