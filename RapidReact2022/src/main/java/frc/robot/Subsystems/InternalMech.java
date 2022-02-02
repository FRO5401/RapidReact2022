package frc.robot.subsystems;

public class InternalMech {
    private WPI_TalonSRX mech1;
    
    public internalMech(){
    mech1 = new WPI_TalonSRX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_1);
    mech1.setInverted(false);
  
}

  

}
