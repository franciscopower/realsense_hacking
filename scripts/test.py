import time

st = "Hello worls"
s = ""
n = 0

for _ in range(50):
   s += ("\n" + st) * (n%5==0) 
   n += 1

print(s)