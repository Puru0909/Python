class Employee:
    a = 1

    @classmethod
    def show(cls):
        print(f"The class attribute 'a' is {cls.a}")

    @property
    def name(self):
        return f"{self.fname} {self.lname}"

    @name.setter
    def name(self, value):
        self.fname = value.split(" ")[0]
        self.lname = value.split(" ")[1] if len(value.split(" ")) > 1 else ""

# Creating an instance of Employee
e = Employee()

# Setting an instance attribute 'a', not affecting the class attribute
e.a = 45

# Setting the name using the property setter
e.name = "Harry Khan"

# Getting the name using the property getter
print(e.name)  # Output: Harry Khan

# Displaying the class attribute 'a' using the class method
e.show()  # Output: The class attribute 'a' is 1
