package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.Constants;
import frc.robot.Utilities.testers.Printer;

import java.lang.Math;

import static frc.robot.Tabs.*;

//TODO: Limit code
public class Climber extends SubsystemBase{
    
    //Creates parts for the robot
    private CANSparkMax transMotor1;
    private CANSparkMax transMotor2;
    private CANSparkMax rotateMotor1;
    private CANSparkMax rotateMotor2;
    private Solenoid ratchetSolenoid;
    private RelativeEncoder tMEncoder1;
    private RelativeEncoder tMEncoder2;
    private RelativeEncoder rMEncoder1;
    private RelativeEncoder rMEncoder2;
    //private DigitalInput limit1;
    //private DigitalInput limit2; 
    private double angle;
    private double startTime;
    private double currentTime;
    private DriveBase driveBase;
    public Climber(DriveBase passedDrivebase) {
        //Instantiates the motors and limits
        transMotor1 = new CANSparkMax(Constants.SubsystemConstants.TRANS_MOTOR_1, MotorType.kBrushless);
        transMotor2 = new CANSparkMax(Constants.SubsystemConstants.TRANS_MOTOR_2, MotorType.kBrushless);
        rotateMotor1 = new CANSparkMax(Constants.SubsystemConstants.ROTATE_MOTOR_1, MotorType.kBrushless);
        rotateMotor2 = new CANSparkMax(Constants.SubsystemConstants.ROTATE_MOTOR_2, MotorType.kBrushless);
        ratchetSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,Constants.SubsystemConstants.RATCHET_SOLENOID);
        tMEncoder1 = transMotor1.getEncoder();
        tMEncoder2 = transMotor2.getEncoder();
        rMEncoder1 = rotateMotor1.getEncoder();
        rMEncoder2 = rotateMotor2.getEncoder();
        //limit1 = new DigitalInput(Constants.SubsystemConstants.DIGITAL_INPUT_1);
        //limit2 = new DigitalInput(Constants.SubsystemConstants.DIGITAL_INPUT_2);
        angle = 0.0;
        //Inverts the necessary motors
        transMotor2.setInverted(true);
        rotateMotor2.setInverted(true);
        ratchetSolenoid.set(false); //default is false, at the end of the game you set true to hold climber position 
        startTime = Timer.getMatchTime();

        driveBase = passedDrivebase;
        rMEncoder1.getPositionConversionFactor();
        rMEncoder1.setPositionConversionFactor(360/42*50); //50:1
        rMEncoder2.getPositionConversionFactor();
        rMEncoder2.setPositionConversionFactor(360/42*50);

        tMEncoder1.getPositionConversionFactor();
        tMEncoder1.setPositionConversionFactor(12); //12:1 gearing
        tMEncoder2.getPositionConversionFactor();
        tMEncoder2.setPositionConversionFactor(12);

        //Makes sure the neutral mode is on brake
        setClimberIdleMode("Translation", IdleMode.kBrake);
        setClimberIdleMode("Climber", IdleMode.kBrake);

        climberShuffleboard();
        resetClimberEncoders();

    }

    //Set motor speeds for the climber
    public void setMotorSpeeds(String type, double speed){
        if(type.toUpperCase().contains("TRANS")){
            transMotor2.set(speed);
            transMotor1.set(speed);
        } else if (type.toUpperCase().contains("ROT")) {
            rotateMotor2.set(speed*0.25);
            rotateMotor1.set(speed*0.25);

        }
    }
    
    //Get motor speeds of the two different climber motors
    public double getMotorSpeeds(String type, int number){
        double returnable = 0;
        if(type.toUpperCase().contains("TRANS")){
            if(number == 1){
                returnable = tMEncoder1.getVelocity();
            } else if (number == 2){
                returnable = tMEncoder2.getVelocity();
            }
        } else if (type.toUpperCase().contains("ROT")) {
            if(number == 1){
                returnable = rMEncoder1.getVelocity();
            } else if (number == 2){
                returnable = rMEncoder2.getVelocity();
            }
        }
        return returnable;
    }
    //2550 is 5'6" from the the ground straight up
    //16800 is 53.8 degrees
    //3460 is 5'8" arm length

    public double posToAngle(double currTransPos){
        double radians = Math.toRadians(currTransPos);
        return radians;
    }
    public boolean checkOverRotExtension(double angle, int currTransPos){
        if(angle > Math.toRadians(2)){
            double tempPosition = (getLeftTransPosition() < 0) ? 0 : getLeftTransPosition() * Constants.SubsystemConstants.climberConversion;
            double horizontalDistance = ((tempPosition * Math.sin(Math.abs(angle))) - Constants.SubsystemConstants.robotFrontOffset);
            if(horizontalDistance >= 16){
                return true;
            }
            else{
                return false;
            }
        }
        else if(angle > Math.toRadians(2)){
            double tempPosition = (getLeftTransPosition() < 0) ? 0 : getLeftTransPosition()* Constants.SubsystemConstants.climberConversion;
            double horizontalDistance = ((tempPosition * Math.sin(Math.abs(angle))) + Constants.SubsystemConstants.robotBackOffset);
            if(horizontalDistance >= 16){
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;
        }


    }
    public boolean checkOverTransExtension(double angle, int currTransPos){
            if(currTransPos >= Constants.SubsystemConstants.climberArmMaxPos){
                Printer.print("Bangerang");
                return true;
            }
            else{
                return false;
            }
    }
    /** 
    public boolean getLimit1(){
        return limit1.get();
    }

    public boolean getLimit2(){
        return limit2.get();
    }
    */
    public void setClimberIdleMode(String type, IdleMode mode){
        if(type.toUpperCase().contains("TRANS")){
            transMotor1.setIdleMode(mode);
            transMotor2.setIdleMode(mode);
        } else if (type.toUpperCase().contains("ROT")) {
            rotateMotor1.setIdleMode(mode);
            rotateMotor2.setIdleMode(mode);
        }
    }

    //Reports the climber sensors periodically
    public boolean getRatchetAirPressure(){
      if( driveBase.getPressureStatus())
        return true;
      else
        return false;
    }

    public void resetClimberEncoders(){
        rMEncoder1.setPosition(0);
        rMEncoder2.setPosition(0);
        tMEncoder1.setPosition(0);
        tMEncoder2.setPosition(0);
    }

    public void setSafetySolenoid(){
        currentTime = Timer.getMatchTime();
        double elapsedTime;
        elapsedTime = currentTime - startTime; 
        if(elapsedTime >= 132 || getRatchetAirPressure() ){
            ratchetSolenoid.set(true);
        }
    }

    public double getLeftRotAngle(){
        return rMEncoder1.getPosition()*0.00344182114;
    }

    public double getRightRotAngle(){
        return rMEncoder2.getPosition()*0.00344182114;
    }
    
    public double getLeftTransPosition(){
        return tMEncoder1.getPosition();
    }

    public double getRightTransPosition(){
        return tMEncoder2.getPosition();
    }
    
    @Override
    public void periodic() {
        reportClimber();
    }

    //Reports the speeds of the climber
    public void reportClimber(){
        SmartDashboard.putNumber("Translation Motor 1 Speed", getMotorSpeeds("TRANSLATION", 1));
        SmartDashboard.putNumber("Translation Motor 2 Speed", getMotorSpeeds("TRANSLATION", 2));
        SmartDashboard.putNumber("Rotation Motor 1 Speed", getMotorSpeeds("ROTATION", 1));
        SmartDashboard.putNumber("Rotation Motor 2 Speed", getMotorSpeeds("ROTATION", 2));
        //SmartDashboard.putBoolean("Airpressure Status Bad", getRatchetAirPressure());
        //SmartDashboard.putBoolean("Limit 1 status", getLimit1());
        //SmartDashboard.putBoolean("Limit 2 status", getLimit2());

        transClimberLeftPositionEntry.setDouble(getLeftTransPosition());
        transClimberRightPositionEntry.setDouble(getRightTransPosition());
        transClimberLeftPositionGraph.setDouble(getLeftTransPosition());
        transClimberRightPositionGraph.setDouble(getRightTransPosition());

        rotClimberLeftAngleEntry.setDouble(getLeftRotAngle());
        rotClimberRightAngleEntry.setDouble(getRightRotAngle());
        rotClimberLeftAngleGraph.setDouble(getLeftRotAngle());
        rotClimberRightAngleGraph.setDouble(getRightRotAngle());
    }

    public void climberShuffleboard(){
        //Testing
        transClimberLeftPositionEntry = testingTab.add("Left T-Climber Position",getLeftTransPosition()).getEntry();
        transClimberRightPositionEntry = testingTab.add("Right T-Climber Position", getRightTransPosition()).getEntry();
        rotClimberLeftAngleEntry = testingTab.add("Left R-Climber Angle", getLeftRotAngle()).getEntry();
        rotClimberRightAngleEntry = testingTab.add("Right R-Climber Angle", getRightRotAngle()).getEntry();


        //Graphing
        transClimberLeftPositionGraph = graphTab.add("Left T-Climber Graph",getLeftTransPosition()).withWidget(BuiltInWidgets.kGraph).getEntry();
        transClimberRightPositionGraph = graphTab.add("Right T-Climber Graph", getRightTransPosition()).withWidget(BuiltInWidgets.kGraph).getEntry();
        rotClimberLeftAngleGraph = graphTab.add("Left R-Climber Graph", getLeftRotAngle()).withWidget(BuiltInWidgets.kGraph).getEntry();
        rotClimberRightAngleGraph = graphTab.add("Right R-Climber Graph", getRightRotAngle()).withWidget(BuiltInWidgets.kGraph).getEntry();
    }
}
