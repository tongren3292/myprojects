package assignment;


import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import lejos.hardware.lcd.LCD;
import colorSensor.ColorSensor;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * Team: C2
 * Team member: Wenyue Jin ; Lushi Yang
 * 
 * p1: follow line
 * p2: green patch...
 * p3: avoid obstacle
 *
 */

public class as1
{
	
	static EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(
			MotorPort.D);
	static EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(
			MotorPort.B);
	
	static EV3ColorSensor cs1 = new EV3ColorSensor(SensorPort.S2);
	static EV3ColorSensor cs2 = new EV3ColorSensor(SensorPort.S3);
	static EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S1);
	
	
    static final int PILOT_SPEED = 2; 
    
	public static void main(String[] args) throws Exception {
		
		float [] sample1;
		float [] sample2;
		float [] sample3;
		
		int rotation;
		
		Wheel wheel1 = WheeledChassis.modelWheel(LEFT_MOTOR, 4.3).offset(-6);
		Wheel wheel2 = WheeledChassis.modelWheel(RIGHT_MOTOR, 4.3).offset(6);               

		//chassis type: Differential pilot
		Chassis chassis = new WheeledChassis(new Wheel [] {wheel1,  wheel2},
				WheeledChassis.TYPE_DIFFERENTIAL);

		MovePilot pilot = new MovePilot(chassis);
		pilot.setLinearSpeed(PILOT_SPEED);
		
        cs1.setFloodlight(Color.WHITE);
        cs2.setFloodlight(Color.WHITE);
		
		SampleProvider color1 = cs1.getColorIDMode();
		SampleProvider color2 = cs2.getColorIDMode();
		SampleProvider distance = irSensor.getDistanceMode();
            
		sample1 = new float[color1.sampleSize()];
    	sample2 = new float[color2.sampleSize()];
    	sample3 = new float[distance.sampleSize()];
    	  	
    	rotation = 4;
        while (Button.ESCAPE.isUp())
        {
        
            color1.fetchSample(sample1, 0);
            color2.fetchSample(sample2, 0);
            distance.fetchSample(sample3, 0);
           
            LCD.drawString("left: "+ colorName((int)sample1[0]),0,1);
            LCD.drawString("right: " + colorName((int)sample2[0]), 0, 2);
            LCD.drawString("Distance: "+sample3[0], 0, 3);
           
            Delay.msDelay(300);
            
            if((sample1[0]==6 || colorName((int)sample1[0])=="Yellow") && (sample2[0]==6 || colorName((int)sample2[0])=="Yellow")) {
        	    pilot.forward();
        	    Delay.msDelay(100);
        	    rotation=4;
        	   
            }
            
            /**
             * sometimes color sensor is on black line but did not detect black,
             * or in the case when it is on white but it is not white.
             */
            if(sample1[0]!=6 && sample1[0]!=1 && (sample2[0]==6 || colorName((int)sample2[0])=="Yellow")) {
        	   
        	    pilot.rotate(-rotation);
        	    pilot.travel(0.5);
        	    Delay.msDelay(100);
        	    
        	    if(sample1[0]==6 && sample2[0]==6) {
            	    pilot.forward();
            	    rotation=4;
            	   
                }      	   
        	    rotation++;
        	   
            }
            
            if((sample1[0]==6 || colorName((int)sample1[0])=="Yellow") && sample2[0]!=6 && sample2[0]!=1) {
        	   
        	    pilot.rotate(rotation);
        	    pilot.travel(0.5);
        	    Delay.msDelay(100);
        	    
        	    if(sample1[0]==6 && sample2[0]==6) {
            	    pilot.forward();
            	    rotation=4;
            	   
                }  
        	    rotation++;
        	   
            } 
            
            /**
             *  the case when two sensors detected black. it will try to move forward and detect if there is line before.
             *  if there is no line in front, it turns back
             *  if there is, it keeps going forward.
             */
            color1.fetchSample(sample1, 0);
            color2.fetchSample(sample2, 0);
            Delay.msDelay(300);
            if(sample1[0]==7 && sample2[0]==7) {
            	pilot.travel(5);
            	pilot.rotate(7);
            	color1.fetchSample(sample1, 0);
            	pilot.rotate(-14);
            	color2.fetchSample(sample2, 0);
            	Delay.msDelay(100);
            	
            	if(sample1[0] == 6 && sample2[0]==6) {
            		pilot.rotate(7);
            		pilot.travel(-5);
            		pilot.rotate(175);
            		pilot.forward();
            	}
            	if(sample1[0]==7 || sample2[0]==7) {
            		pilot.rotate(5);
            		pilot.forward();
            		
            	}
            	
            }
            
           /**
            * In the case when it did not detect black and white at all.
            */
            if(sample1[0]!=6 && sample2[0]!=6 && sample1[0]!=7 && sample2[0]!=7) {
        	   
        	    if(sample1[0]>sample2[0]) {
        		    
        		    pilot.rotate(rotation);
        		    pilot.travel(0.5);
        		    Delay.msDelay(100);
        		    if(sample1[0]==6 && sample2[0]==6) {
                	    pilot.forward();
                	    rotation=4;
                	   
                    }  
        		    
        	    }
                if(sample1[0]<sample2[0]) {
        		    
        		    pilot.rotate(-rotation);
        		    pilot.travel(0.5);
        		    Delay.msDelay(100);
        		    
        		    if(sample1[0]==6 && sample2[0]==6) {
                	    pilot.forward();
                	    rotation=4;
                	   
                    }  
        		    
        	    }  
        	    if(sample1[0]==sample2[0]) {
        		   pilot.forward();
        		   rotation=4;
        	    }
        	    rotation++;
            }
            
            /**
             * detect green.
             * case1: left and right are both green, turn back
             * case2: left is green
             * case3: right is green
             */
            color1.fetchSample(sample1, 0);
            color2.fetchSample(sample2, 0);
            Delay.msDelay(500);
            if(sample1[0]==1 && sample2[0]==1) {
         	   
        	    pilot.rotate(180);
        	  
            }
            
            if(sample1[0]==1 && sample2[0]==6) {
        	 
               pilot.travel(6);
               Delay.msDelay(50);
               pilot.rotate(-76);
               pilot.travel(6);
              
            }
            if(sample1[0]==6 && sample2[0]==1) {
        	    pilot.travel(6);
        	    Delay.msDelay(50);
                pilot.rotate(76);
                pilot.travel(6);
 	   
            }
            
            //when IR sensor detects an obstacle in front
            if(sample3[0]<15) {
            	pilot.rotate(80);
            	pilot.travel(16);
            	pilot.rotate(-76);
            	pilot.travel(37);
            	pilot.rotate(-76);
            	pilot.travel(16);
            	pilot.rotate(80);
            	
            }
            
            /**
             * one of the two color sensors detect red does not effect,
             * both of them detect red, the robot will stop and terminate.
             */
            color1.fetchSample(sample1, 0);
            color2.fetchSample(sample2, 0);
            Delay.msDelay(500);
            if(sample1[0]==0 && sample2[0]==0) {
            	pilot.stop();
            	break;
            }
            if(sample1[0]==0) {
            	pilot.forward();
            }
            if(sample2[0]==0) {
            	pilot.forward();
            }
            
            LCD.clear();
         }
		
	 }
	
	 public static String colorName(int color)
	 {
		 switch (color)
		 {
		 	 case Color.NONE:
				 return "None";
				
			 case Color.BLACK:
				 return "Black";
				
			 case Color.BLUE:
				 return "Blue";
				
			 case Color.BROWN:
				 return "Brown";
				
			 case Color.CYAN:
				 return "Cyan";
				
			 case Color.DARK_GRAY:
				 return "Dark Gray";
				
			 case Color.GRAY:
				 return "Gray";
				
			 case Color.GREEN:
				 return "Green";
				
			 case Color.LIGHT_GRAY:
				 return "Light Gray";
				
			 case Color.MAGENTA:
			  	 return "Magenta";
				
			 case Color.ORANGE:
				 return "Orange";
				
			 case Color.PINK:
				 return "Pink";
				
			 case Color.RED:
				 return "Red";
				
			 case Color.WHITE:
				 return "White";
			 	
			 case Color.YELLOW:
				 return "Yellow";
		 }
		
		 return "";
	 }
}
