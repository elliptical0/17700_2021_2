package org.firstinspires.ftc.teamcode;

import java.util.*;

import static org.firstinspires.ftc.teamcode.MyMathLib.*;

 public class MyTest {
     public static void main(String[] args) {
         int i = 0;
         int[] colorSensorHistory = {2, 2, 2, 2, 1, 3};
         int stackSize;

         int[] tally = new int[3];
         for(int n : colorSensorHistory) {
             if(n > 0) {
                 tally[n - 1] = tally[n - 1] + 1;
             }
         }
         int highest = 0;
         for(i = 1; i < tally.length; i++) {
             if(tally[i] > tally[highest]) {
                 highest = i;
             }
         }
         stackSize = highest + 1;

         System.out.println("StackSize: " + stackSize);
         System.out.println("Tally: " + tally[0] + ", " + tally[1] + ", " + tally[2]);
     }
 }