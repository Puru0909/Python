def pattern(n):
    if(n==0):
       return 0
    print("*" * n)
    pattern(n-1)

a = int(input("enter the value of n: "))


print(pattern(a))