package org.firstinspires.ftc.teamcode;

import java.util.*;

import static org.firstinspires.ftc.teamcode.MyMathLib.*;

 public class MyTest {
     public static void main(String[] args) {
         Vector2 v = new Vector2(1, 1);
         System.out.println(updatePosition(new Transform(0, 0, 0), new double[3], new double[3]));
     }
 }