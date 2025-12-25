class Employee:
    language = "Pyhton"
    salary = 120000

    def __init__(self, name, salary, language):
        self.name = name
        self.salary = salary
        self.language = language
        print("I am creating an object")

    def getInfo(self):
        print(f"the language is {self.language}. The salary is {self.salary}")


    
    def greet(self):
        print("good morning")
            


harry = Employee("Harry", 130000, "Javascript")
# harry.name = "Harry"
print(harry.name, harry.salary, harry.language)

# rohan = Employee()