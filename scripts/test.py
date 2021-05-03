#!/usr/bin/env python3

from time import sleep

def main():
   f = open("test.txt", "w") 
   for i in range(10):
      f.write(str(i))
   f.close
      
if __name__ == "__main__":
   main()