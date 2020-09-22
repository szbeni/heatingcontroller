#!/usr/bin/python3

import sys

sys.argv[1:]

if len(sys.argv) < 2:
    print("Off")
    exit()

inp = sys.argv[1]
if(inp == '00000'):
     print ("Off")
elif(inp == '10000'): 
    print ("Gas Auto Fan")
elif(inp == '01000'): 
    print ("Gas Slow Fan");
elif(inp == '00100'):
    print ("Fan");
elif(inp == '00010'):
    print ("Elec Auto Fan");
elif(inp == '00001'):
    print ("Elec Slow Fan");
else:
    print("Error " + sys.argv[1])
