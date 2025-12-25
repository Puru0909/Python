class Employee:
    company = "ITC"
    name = "Default name"
    def show(self):
        print(f"The name of the employee is {self.name} and the company is {self.company}")

class Coder:
    language = "Python"
    def printLanguages(self):
        print(f"Out of all the languages, here is your language: {self.language}")

class Programmer(Employee, Coder):
    company = "ITC Infotech"  
    def showLanguage(self):    
        print(f"The name is {self.name} and he is good with {self.language}")

a = Employee()
b = Programmer()

b.show()            # Inherited from Employee
b.printLanguages()  # Inherited from Coder
b.showLanguage()    # Method in Programmer, accessing both Employee and Coder attributes
