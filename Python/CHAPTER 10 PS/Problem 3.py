class Demo:
    a = 4

o = Demo()
print(o.a) #Prints the class Attribute because  instances attribute is not present

o.a = 0  #instance attribute is set
print(o.a) #prints the instance attribute because instance attribute is present
print(Demo.a)    #Prints the class attribute