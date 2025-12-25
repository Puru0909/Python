a = int(input("enter your age: "))

# if Statement no 1
if(a%2 == 0):
    print("a is even")
    # End of if statement no 1


#if statement no 2

if(a>=18):
    print("you are legal")
elif(a<0):
    print("you are entering an invalid age")   
elif(a==0):
    print("invalid age") 
else:
    print("you are below legal")



print("end of program")
