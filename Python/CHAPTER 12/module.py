def myFunc():
    print("hello world")

if __name__=="__Main__":
    #if this code is already executed by running the file its present in
    print("We are directly running this code")
    myFunc()
    print(__name__)