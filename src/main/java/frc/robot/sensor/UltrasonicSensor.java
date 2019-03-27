package frc.robot.sensor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UltrasonicSensor {
    
    private final static String DISTANCE_KEY = "distance";
    // The handle to access the sensor
    private final AnalogInput proximityFinder;
    
    // The scaling factor:  distance in inches = volts returned / SCALING_FACTOR
    private final double SCALING_FACTOR = 1/20.5;
    
    /** Creates a new ultrasonic sensor hooked up to <code>portNumber</code> on the analog breakout.
     * @params portNumber The port number on the breakout.
     */
    public UltrasonicSensor(int portNumber){
        proximityFinder = new AnalogInput(portNumber);
        proximityFinder.setAccumulatorInitialValue(0);
    }
    
    /** Returns the distance measured in inches.  */
    public double getInches(){
        double volts = proximityFinder.getAverageVoltage();
        return (double) (volts * SCALING_FACTOR);
    }
    
    /** Returns the distance measured in feet.  */
    public double getFeet(){
        return this.getInches() / 12.0;
    }
    
    /** Returns the distance measured as "feet<code>'</code> inches<code>"</code>".  */
    
    public String getReadable(){
        return (int)getFeet() + "' " + (int)(getInches() - (int)getFeet()*12) + "\"";
    }
    
    public void setLightState() {
        //takes a relay as input and changes the values depending on distance to target
        
        if(this.getInches() > 45 && this.getInches() < 51) {
            SmartDashboard.putString(DISTANCE_KEY, "Red");
        }
        else if(this.getInches() > 69 && this.getInches() < 75) {
            SmartDashboard.putString(DISTANCE_KEY, "Yellow");
        }
        else if(this.getInches() > 93 && this.getInches() < 99) {
            SmartDashboard.putString(DISTANCE_KEY, "Dark Green");
        }
        else {
            SmartDashboard.putString(DISTANCE_KEY, "Green");
        }
    }
}