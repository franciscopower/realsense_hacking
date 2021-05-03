#!/usr/bin/env python3

from time import sleep

def main():
   f = open("/home/franciscopower/catkin_ws/src/realsense_hacking/OldCameraTrajectory.txt", "r") 

   for l in f.read().split("\n"):
      data = l.split(" ")
      # print("line: \"" + data[0] + "\"")
      if len(data) == 8:
         print(data)
      
   
   f.close

      
if __name__ == "__main__":
   main()