package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables extends SubsystemBase {
  
  NetworkTable ballTable;
  NetworkTableInstance inst;
  NetworkTableEntry ballXEntry, ballYEntry, ballDEntry, ballREntry;
  private static double ballX, ballY, ballDistance;
  public double radius;


  public NetworkTables() {
    inst = NetworkTableInstance.getDefault();
    ballTable = inst.getTable("Ball");
    ballXEntry = ballTable.getEntry("cX");
    ballYEntry = ballTable.getEntry("cY");
    ballDEntry = ballTable.getEntry("distance");
    ballREntry = ballTable.getEntry("radius");
    ballX = 0.0;
    ballY = 0.0;
    ballDistance = 0.0;
    radius = 0.0;


    inst.startClientTeam(5401); // where TEAM=190, 294, etc, or use inst.
    inst.startDSClient();

    Shuffleboard.getTab("SmartDashboard").add("Current Ball X", getBXValue()).withWidget(BuiltInWidgets.kGraph);
    Shuffleboard.getTab("SmartDashboard").add("Current Ball Y", getBYValue()).withWidget(BuiltInWidgets.kGraph);
    Shuffleboard.getTab("SmartDashboard").add("Current Ball radius", getBallRadius()).withWidget(BuiltInWidgets.kGraph);
    Shuffleboard.getTab("SmartDashboard").add("Ball Distance", getBallDistance()).withWidget(BuiltInWidgets.kGraph);
  }

  @Override
  public void periodic() {
    updateValue();
    reportValues();
    //odometry.update(navxGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  public void updateValue() {
     // recommended if running on DS computer; this gets the robot

    ballX = ballXEntry.getDouble((ballX != 0) ? ballX : 0);
    ballY = ballYEntry.getDouble((ballY != 0) ? ballY : 0);
    radius = (ballX == 0 && ballY == 0) ? 0 : ballREntry.getDouble((radius != 0) ? radius : 0);
    ballDistance = ballDEntry.getDouble((ballDistance != 0) ? ballDistance : 0);
    //powerPortX = powerPortXEntry.getDouble(0.0);
    //powerPortY = powerPortYEntry.getDouble(0.0);
    //System.out.println("The Ball coordinates are: " + "X: " + ballX + " Y: " + ballY);
    //System.out.println("The Ball is " + ballDistance + " away");
    //System.out.println("The radius is " + radius);
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

    if(getBXValue() >= 300 && getBXValue() <= 340){
      return true;
    }
    
    else if(((getBXValue() < 300) & (getBXValue() >= 0)) || ((getBXValue() > 340) & (getBXValue() <= 640))){
      return false;
    }
    return false;
  }



  public void reportValues()
  {
   
  }
}