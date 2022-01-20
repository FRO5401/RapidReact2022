package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class NetworkTables extends SubsystemBase {
  
  NetworkTable ballTable;
  NetworkTableInstance inst;
  NetworkTableEntry ballXEntry, ballYEntry, ballDEntry, ballREntry;
  private static double ballX, ballY, ballDistance;
  public double radius;


  public NetworkTables() {
    ballX = 0;
    ballY = 0;
    ballDistance = 0;
    radius = 0;
  }

  @Override
  public void periodic() {
    updateValue();
    reportValues();
    
    //odometry.update(navxGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  public void updateValue() {
    inst = NetworkTableInstance.getDefault();
    ballTable = inst.getTable("Ball");
    ballXEntry = ballTable.getEntry("cX");
    ballYEntry = ballTable.getEntry("cY");
    ballDEntry = ballTable.getEntry("distance");
    ballREntry = ballTable.getEntry("radius");


    inst.startClientTeam(5401); // where TEAM=190, 294, etc, or use inst.
    inst.startDSClient(); // recommended if running on DS computer; this gets the robot

    ballX = ballXEntry.getDouble(0.0);
    ballY = ballYEntry.getDouble(0.0);
    radius = (ballX == 0 && ballY == 0) ? 0 : ballREntry.getDouble(0.0);
    ballDistance = ballDEntry.getDouble(0.0);
    //powerPortX = powerPortXEntry.getDouble(0.0);
    //powerPortY = powerPortYEntry.getDouble(0.0);
    System.out.println("The Ball coordinates are: " + "X: " + ballX + " Y: " + ballY);
    System.out.println("The Ball is " + ballDistance + " away");
    System.out.println("The radius is " + radius);
    //System.out.println("The Power Port coordinates are: " + "X: " + powerPortY + " Y: " + powerPortY);
  }

  public double getBXValue() {
    return ballX;
  }

  public double getBYValue() {
    return ballY;
  }

  public double getBallDistance(){
    return ballDistance;
  }

  public double getBallRadius(){
    return radius;
  }

  public void resetValues(){
    ballX = 0;
    ballY = 0;
    ballDistance = 0;
    radius = 0;
  }

  public boolean checkCentered(){

    if(getBXValue() >= 350 && getBXValue() <= 450){
      return true;
    }
    
    else if(((getBXValue() < 350) & (getBXValue() > 0)) || ((getBXValue() > 450) & (getBXValue() < 800))){
      return false;
    }
    return false;
  }



  public void reportValues()
  {
    SmartDashboard.putNumber("Current Ball X", getBXValue());
    SmartDashboard.putNumber("Current Ball Y", getBYValue());
    SmartDashboard.putNumber("Current Ball radius", getBallRadius());
    SmartDashboard.putNumber("Ball Distance", getBallDistance());
  }
}