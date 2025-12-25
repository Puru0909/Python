with open("C:/Users/PURUSHOTTAM/Desktop/Python/CHAPTER 9 PS/file.txt") as f:
    content1 = f.read()

with open("C:/Users/PURUSHOTTAM/Desktop/Python/CHAPTER 9 PS/hiscore.txt") as f: 
    content2 = f.read()

if(content1 == content2):
    print("yes these files are identical")

else:
    print("No these files are not identical")        