package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CompressorSubsystem extends SubsystemBase{
    
    private Compressor compressor;

	public CompressorSubsystem() {
        //No idea what a pneumaticsmoduletype be
		compressor = new Compressor(PneumaticsModuleType.REVPH);	
	}


    @Override
    public void periodic() {
        reportCompressorStatus();
    }

    public void startCompressor() {
    	compressor.enableDigital();
    }
    
    public void stopCompressor() {
		compressor.disable();
    }

    public void reportCompressorStatus(){
    	SmartDashboard.putBoolean("Compressor Enabled", compressor.enabled());
    	SmartDashboard.putNumber("Compressor Current Value", compressor.getCurrent());
    	SmartDashboard.putBoolean("Compressor Pressure Switch On/Off", compressor.getPressureSwitchValue());
    }
    
    public boolean isEnabled(){
    	return compressor.enabled();
    }

}