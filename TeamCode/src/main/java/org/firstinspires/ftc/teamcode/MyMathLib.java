package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;

// Math reference: https://seamonsters-2605.github.io/archive/mecanum/

 public class MyMathLib {
     /**
      * @author TM
      * @param v
      * @param min
      * @param max
      * @return
      */
     public static double clamp(double v, double min, double max) {
         return Math.max(min, Math.min(max, v));
     }

     /**
      * @author TM
      * @param v
      * @param min
      * @return
      */
     public static double absMin(double v, double min) {
         if(Math.abs(v) < min) {
             if(v >= 0) {
                 return min;
             } else {
                 return -min;
             }
         } else {
             return v;
         }
     }

     /**
      * @author TM
      * @param t0 theta
      * @return Smallest co-terminal angle
      */
     public static double coterminal(double t0) {
         double t1 = t0 % (Math.PI * 2);
         if(t1 > Math.PI) {
             t1 -= (Math.PI * 2);
         }
         return t1;
     }

     /**
      * Turn controller input into motor output for a Mecanum drive
      * @author TM
      * @param x Joystick X-axis, -1 to 1
      * @param y Joystick Y-axis, -1 to 1
      * @param r Rotational magnitude, -1 to 1
      * Rotational power will take 100% priority over directional power.
      * Note: Right-side motors are inverted. If this code does not work, change the inversion before anything else.
      */
     public static double[] mecMath(double x, double y, double r) {
         double[] wheelPowers = new double[4];
         //double mag = Math.min(1.0, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))); // Magnitude
         double mag = Math.max(-1.0, Math.min(1.0, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)))); // Magnitude
         double theta = Math.atan2(y, -x);
         double powerA = clamp(Math.sqrt(2) * Math.sin(theta - 0.25 * Math.PI), -1, 1) * mag;
         double powerB = clamp(Math.sqrt(2) * Math.sin(theta + 0.25 * Math.PI), -1, 1) * mag;
         double dirMag = 1.0 - Math.abs(r); // Directional Magnitude
         double rotMag = r; // Rotational Magnitude

         wheelPowers[0] = (rotMag + powerB * dirMag); // Front-left
         wheelPowers[1] = (rotMag - powerA * dirMag); // Front-right
         wheelPowers[2] = (rotMag - powerB * dirMag); // Back-right
         wheelPowers[3] = (rotMag + powerA * dirMag); // Back-left

         return wheelPowers;
     }

     /**
      * @author TM
      * @param t0 Current transform
      * @param t1 Target transform
      * @return Wheel powers to drive toward the target position.
      */
     public static double[] seekLocation(Transform t0, Transform t1) {
         Vector2 v = new Vector2(0, 0);
         double r = 0;
         Vector2 dir = t1.pos.subtract(t0.pos);
         if(dir.length() > DEADZONE_POS) {
             v = dir.rotate(coterminal(t0.head)).divide(6, 6);
         }
         if(Math.abs(t1.head - coterminal(t0.head)) > DEADZONE_ANGLE) {
             r = absMin(clamp(t1.head - coterminal(t0.head), -1, 1), 0.2);
         }
         return mecMath(v.y, -v.x, r); //For the old bot, v.x is negative.
     }

     /**
      * @author TM
      * @param ticks
      * @return inches
      */
     public static double encoderToInch(double ticks) {
         return DEADWHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
     }

    /**
     * Update estimated position using the dead wheel encoders
     * @author TM
     * @param t Previous transform
     * @param denc Delta of encoders
     * @param penc Position of encoders
     * @return New position
     * This is the odometry updating function.
     */
     public static Transform updatePosition(Transform t, double[] denc, double[] penc) {
         int i;
         double[] dwheel = new double[3];
         for(i = 0; i < 3; i++) {
             dwheel[i] = encoderToInch(denc[i]);
         }
         double[] pwheel = new double[2];
         for(i = 0; i < 2; i++) {
             pwheel[i] = encoderToInch(penc[i]);
         }
         double dhead = (dwheel[0] - dwheel[1]) / LATERAL_DISTANCE;
         Vector2 dpos;

         /*
         if(dhead == 0) {
             dpos = new Vector2(dwheel[2], dwheel[0]);
         } else {
             dpos = new Vector2(((dwheel[2] - ((1 - dhead) * REAR_OFFSET)) / dhead) + REAR_OFFSET, (dwheel[0] / dhead) + (LATERAL_DISTANCE / 2));
             dpos = dpos.multiply(2 * Math.sin(dhead / 2)).rotate(-(dhead / 2));
         }
          */

         double dx = (dwheel[0] + dwheel[1]) / 2;
         double dy = (dwheel[2]) - dhead * REAR_OFFSET;

         /*
         dpos = new Vector2(dx * Math.cos(t.head), dx * Math.sin(t.head));
         dpos.set(dpos.add(dy * -Math.sin(t.head), dy * Math.cos(t.head)));
          */

         dpos = new Vector2(dx, dy);
         dpos = dpos.rotate(-(dhead / 2 + t.head));

         //t.pos.set(t.pos.add(dpos));
         //t.head += dt;
         return new Transform(t.pos.add(dpos), (pwheel[0] - pwheel[1]) / LATERAL_DISTANCE);
     }
 }