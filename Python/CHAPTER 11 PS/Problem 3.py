class Employee:
    salary = 234
    increment = 20
    
    @property
    def salaryAfterIncrement(self):
        return (self.salary + self.salary * (self.increment / 100))
    
    @salaryAfterIncrement.setter
    def salaryAfterIncrement(self, salary):
        self.increment = ((salary/self.salary) -1)*100

# Create an instance of Employee
e = Employee()

# Print the salary after increment
# print(e.salaryAfterIncrement)  # Output: 280.8

e.salaryAfterIncrement = 280.8
print(e.increment)
