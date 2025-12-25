try:
    a = int(input("hey, Enter a number: "))

    print(a)

except ValueError as v:
      print("HEyy")
      print(v)    

except  Exception as e:
        print(e)

print("Thank you ")        