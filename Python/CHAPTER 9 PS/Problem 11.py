with open("C:/Users/PURUSHOTTAM/Desktop/Python/CHAPTER 9 PS/old.txt") as f:
    content = f.read()

with open("C:/Users/PURUSHOTTAM/Desktop/Python/CHAPTER 9 PS/renamed_by_pyhton.txt","w") as f:
    f.write(content)