p1 = "Make a lot of money"
p2 = "Make a lot of money"
p3 = "Make a lot of money"
p4 = "Make a lot of money"

message = input("Enter your comment: ")
if((p1 in message) or (p2 in message) or (p3 in message) or (p4 in message)):
    print("this comment is a spam")

else:
    print("this comment is not a spam")