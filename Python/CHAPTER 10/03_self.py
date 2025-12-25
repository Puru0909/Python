class Employee:
    language = "Pyhton"
    salary = 120000

    def getInfo(self):
        print(f"the language is {self.language}. The salary is {self.salary}")


    
    def greet(self):
        print("good morning")
            


harry = Employee()
# harry.language = "JavaScript"    
# print(harry.language, harry.salary)  
harry.getInfo()
harry.greet()
# Employee.getInfo(harry)  