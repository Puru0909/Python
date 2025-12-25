f = open("C:/Users/PURUSHOTTAM/Desktop/Python/CHAPTER 9/file.txt")
print(f.read())
f.close()

# the same can be written using with statement like this
with open("C:/Users/PURUSHOTTAM/Desktop/Python/CHAPTER 9/file.txt") as f:
    print(f.read())

    # you dont have to explicitly close the file