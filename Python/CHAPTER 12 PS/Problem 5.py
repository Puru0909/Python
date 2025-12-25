n = int(input("Enter a number: "))
table = [n*i for i in range(1, 11)]

# Using a raw string to handle the backslashes in the file path
with open(r"C:\Users\PURUSHOTTAM\Desktop\Python\CHAPTER 12 PS\tables.txt", "a") as f:
    f.write(f"Table of {n}: {str(table)} \n")
