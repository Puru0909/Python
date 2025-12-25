a = 89
def fun():
    global a
    #global a
    a = 3
    print(a)


fun()
print(a)    