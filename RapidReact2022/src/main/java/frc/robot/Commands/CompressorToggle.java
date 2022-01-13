package frc.robot.Commands;

import frc.robot.Subsystems.CompressorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls;

public class CompressorToggle extends CommandBase {
    
    CompressorSubsystem compressor;
    Controls controls;

    public CompressorToggle(CompressorSubsystem m_compressor, Controls m_controls) {
        // Use requires() here to declare subsystem dependencies
        compressor = m_compressor;
        addRequirements(compressor);
    }

    // Called just before this Command runs the first time
    @Override
	public void initialize() {
    //    
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	public void execute() {
        if(compressor.isEnabled()){
            compressor.stopCompressor();
        } 
        else {
            compressor.startCompressor();
        }
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}