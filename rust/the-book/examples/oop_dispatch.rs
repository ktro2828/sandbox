trait Animal {
    fn speak(&self);
}

struct Dog;
impl Animal for Dog {
    fn speak(&self) {
        println!("Woof!");
    }
}

struct Cat;
impl Animal for Cat {
    fn speak(&self) {
        println!("Meow!");
    }
}

/// Static dispatch function to call `speak` from `Animal`.
fn speak_animal<T: Animal>(animal: &T) {
    animal.speak();
}

/// Dynamic dispatch function to call `speak` from `Animal`.
fn speak_animal_dyn(animal: &dyn Animal) {
    animal.speak();
}

fn main() {
    let dog = Dog;
    let cat = Cat;

    // static dispatch
    speak_animal(&dog);
    speak_animal(&cat);

    let animals: Vec<Box<dyn Animal>> = vec![Box::new(Dog), Box::new(Cat)];

    // dynamic dispatch
    for animal in animals {
        speak_animal_dyn(&*animal);
    }
}
