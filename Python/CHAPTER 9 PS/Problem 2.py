import random

def game():
    print("you are playing the game...")
    score = random.randint(1, 62)
    # fetch the hiscore
    with open("C:/Users/PURUSHOTTAM/Desktop/Python/CHAPTER 9 PS/hiscore.txt") as f:
        hiscore = f.read()
        if(hiscore!=""):
            hiscore = int(hiscore)
        else:
            hiscore = 0
    print(f"Your score: {score}"  )
    if(score>hiscore):
        #write this hiscore to the file
        with open("C:/Users/PURUSHOTTAM/Desktop/Python/CHAPTER 9 PS/hiscore.txt", "w") as f:
            f.write(str(score))

    return score


game()
