f = open("C:/Users/PURUSHOTTAM/Desktop/Python/CHAPTER 9 PS/poem.txt")
content = f.read()

if("twnikle" in content):
    print("the word twinkle is present in the content")
else:
    print("the word twinkle is not present in the content")  

f.close()      