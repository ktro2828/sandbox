import hello


def main():
    print(hello.add(1, 2))
    print(hello.sub(i=1, j=2))
    print(hello.mul())

    dog = hello.Dog("john", 10)
    print(dog.bark())
    print(dog.name, dog.age)
    # set age
    dog.set(20)
    # set name
    dog.set("bob")
    print(dog.name, dog.age)


if __name__ == "__main__":
    main()
