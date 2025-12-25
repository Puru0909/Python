word = "Donkey"

with open("C:/Users/PURUSHOTTAM/Desktop/Python/CHAPTER 9 PS/file.txt", "r") as f:
    content = f.read()

contentNew = content.replace(word, "######")

with open("C:/Users/PURUSHOTTAM/Desktop/Python/CHAPTER 9 PS/file.txt", "w") as f:
     f.write(contentNew)