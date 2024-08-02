package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.RainbowAnimation;

public class LEDStrip extends SubsystemBase {
    CANdle candle;
    CANdleConfiguration config;
    RainbowAnimation animation;
    public LEDStrip() {
        candle = new CANdle(22,"Drive");
        config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.5;
        candle.configAllSettings(config);
        candle.configV5Enabled(true);
        animation = new RainbowAnimation(1, 1, 50);
        candle.setLEDs(0, 0, 0);
        // candle.animate(animation);
        candle.setLEDs(240, 172, 0, 0, 0, 70);
        
    }

    @Override
    public void periodic() {
        
    }
}
